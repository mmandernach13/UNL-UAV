#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleControlMode
from uav_interfaces.action import GoToWaypoint
import threading
import time
import math
import numpy as np

class PositionController(Node):
    def __init__(self): 
        super().__init__('position_control')
        
        # QoS Profile - Quality of Service settings for reliable PX4 communication
        # BEST_EFFORT: Don't guarantee delivery (faster, PX4 uses this)
        # TRANSIENT_LOCAL: New subscribers get last published message
        # KEEP_LAST with depth=1: Only keep most recent message
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # State variables to track UAV status
        self.uav_position = []  # Current [x, y, z] position in local NED frame
        self.take_off = False   # Flag: Has UAV reached takeoff altitude?
        self.offboard_enable = False  # Flag: Is offboard mode active?
        self.armed = False  # Flag: Are motors armed?

        self.declare_parameter('loop_rate', 10.0)
        self.loop_freq = self.get_parameter('loop_rate').value
        self.rate = self.create_rate(self.loop_freq)

        self.action_server = ActionServer(
            self, 
            GoToWaypoint,
            'go_to_waypoint',
            self.excecute_waypoint_cb,
            qos_profile=qos_profile
        )
        
        self.declare_parameter('offboard_rate', 5.0)
        self.offboard_rate = self.get_parameter('offboard_rate').value 

        self.maintain_offboard_timer = self.create_timer(1/self.offboard_rate, self.maintain_offboard_cb)

        # SUBSCRIBERS - Listen to PX4 status and position
        # Vehicle position in local North-East-Down (NED) coordinate frame
        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition, 
            '/fmu/out/vehicle_local_position_v1', 
            self.vehicle_local_position_cb, 
            qos_profile
        )
        
        # Vehicle control mode (tells us if offboard mode is active)
        self.vehicle_control_mode_sub = self.create_subscription(
            VehicleControlMode, 
            '/fmu/out/vehicle_control_mode', 
            self.vehicle_control_mode_cb, 
            qos_profile
        )
        
        # Vehicle status (arming state, flight mode, etc.)
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, 
            '/fmu/out/vehicle_status_v1', 
            self.status_cb, 
            qos_profile
        )
        
        # PUBLISHERS - Send commands and setpoints to PX4
        # Send high-level commands (arm, disarm, mode changes, land, etc.)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, 
            '/fmu/in/vehicle_command', 
            qos_profile
        )
        
        # Tell PX4 what control mode we want (position, velocity, attitude, etc.)
        # Must publish this at >2Hz or PX4 will exit offboard mode
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, 
            '/fmu/in/offboard_control_mode', 
            qos_profile
        )
        
        # Send position/velocity setpoints for the UAV to follow
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, 
            '/fmu/in/trajectory_setpoint', 
            qos_profile
        )

    def send_offboard_signal(self):
        offboard_signal_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(offboard_signal_msg)

    def enter_offboard_mode(self):
        # Create offboard control mode message
        # This tells PX4 we want to control POSITION (not velocity/attitude/etc)
        self.get_logger().info('Going into OFFBOARD mode')
        global offboard_signal_msg 
        offboard_signal_msg = OffboardControlMode()
        offboard_signal_msg.position = True  # We'll send position setpoints
        offboard_signal_msg.velocity = False
        offboard_signal_msg.acceleration = False
        offboard_signal_msg.attitude = False
        offboard_signal_msg.body_rate = False

        self.send_offboard_signal()

        itr = 0

        while(self.offboard_enable == False):
            print("Wait offboard mode")
            # Wait a bit before requesting (let offboard signal propagate)
            if(itr > 10):
                cmd_msg = VehicleCommand()
                cmd_msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
                cmd_msg.param1 = 1.0  # Custom mode enabled
                cmd_msg.param2 = 6.0  # Offboard mode (PX4_CUSTOM_MAIN_MODE_OFFBOARD)
                cmd_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                self.vehicle_command_publisher.publish(cmd_msg)
            itr = itr + 1 
            self.rate.sleep()

        self.get_logger().info('OFFBOARD mode enabled')

    def arm_uav(self):
        self.get_logger().info('Arming UAV')
        cmd_msg = VehicleCommand()
        cmd_msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd_msg.param1 = 1.0  # 1.0 = arm, 0.0 = disarm
        
        while (self.armed == False):
            print('waiting to arm')
            cmd_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.vehicle_command_publisher.publish(cmd_msg)
            self.rate.sleep()

        self.get_logger().info('UAV armed')

    def excecute_waypoint_cb(self, goal: GoToWaypoint):

        if (self.offboard_enable == False):
            self.enter_offboard_mode()

        if (self.armed == False):
            self.arm_uav()

        

        return result
   
    def main_loop(self):
        """
        Main control loop that:
        1. Switches to offboard mode
        2. Arms the vehicle
        3. Takes off to altitude
        4. Flies through waypoints
        5. Lands when mission complete
        """
        rate = self.create_rate(10)  # Run at 10 Hz
        
        # Wait for initial position from PX4 before starting
        print("Wait for uav initial position...")
        while (len(self.uav_position) == 0):
             time.sleep(0.1)
        print("... position received")

        # Calculate target altitude (10 meters below current z)
        # NED frame: z is DOWN, so subtracting makes us go UP
        z_takeoff = self.uav_position[2] - 10.0

        # Create offboard control mode message
        # This tells PX4 we want to control POSITION (not velocity/attitude/etc)
        offboard_signal_msg = OffboardControlMode()
        offboard_signal_msg.position = True  # We'll send position setpoints
        offboard_signal_msg.velocity = False
        offboard_signal_msg.acceleration = False
        offboard_signal_msg.attitude = False
        offboard_signal_msg.body_rate = False

        # Template for vehicle commands (arm, mode change, land)
        cmd_msg = VehicleCommand()
        cmd_msg.target_system = 1  # System ID (usually 1 for single vehicle)
        cmd_msg.target_component = 1  # Component ID (1 = autopilot)
        cmd_msg.source_system = 1  # Our system ID
        cmd_msg.source_component = 1  # Our component ID
        cmd_msg.from_external = True  # Command from external source (not GCS)

        # Mission state tracking
        wp_itr = 0  # Current waypoint index
        itr = 0  # Iteration counter for initial offboard mode request
        mission_done = False  # Has mission completed?
        to_exit = False  # Should we exit the loop?

        while to_exit == False:
            # CRITICAL: Must publish offboard control mode at >2Hz
            # If we stop publishing, PX4 will exit offboard mode for safety
            offboard_signal_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.offboard_control_mode_publisher.publish(offboard_signal_msg)
            
            # STEP 1: Request offboard mode if not already enabled
            if(self.offboard_enable == False):
                print("Wait offboard mode")
                # Wait a bit before requesting (let offboard signal propagate)
                if(itr > 10):
                    cmd_msg = VehicleCommand()
                    cmd_msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
                    cmd_msg.param1 = 1.0  # Custom mode enabled
                    cmd_msg.param2 = 6.0  # Offboard mode (PX4_CUSTOM_MAIN_MODE_OFFBOARD)
                    cmd_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                    self.vehicle_command_publisher.publish(cmd_msg)
                itr = itr + 1 
        
            # STEP 2: Arm the vehicle once offboard mode is active
            # PX4 requires offboard mode BEFORE arming for safety
            if(self.offboard_enable):
                cmd_msg = VehicleCommand()
                cmd_msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
                cmd_msg.param1 = 1.0  # 1.0 = arm, 0.0 = disarm
                cmd_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                self.vehicle_command_publisher.publish(cmd_msg)

            # STEP 5: Land when mission is complete
            if(mission_done == True):
                cmd_msg.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
                cmd_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                self.vehicle_command_publisher.publish(cmd_msg)
                to_exit = True
            else:
                # STEP 3 & 4: Send position setpoints once armed
                if(self.armed):
                    msg = TrajectorySetpoint()
                    
                    # Check if we've reached takeoff altitude (within 0.3m)
                    if(math.fabs(self.uav_position[2] - z_takeoff) < 0.3):
                        self.take_off = True
                    
                    # If at takeoff altitude, start waypoint navigation
                    if (self.take_off == True):
                        if(wp_itr < len(self.waypoints)):
                            # Send current waypoint as position setpoint
                            msg.position = [self.waypoints[wp_itr][0], 
                                          self.waypoints[wp_itr][1], 
                                          z_takeoff]
                            msg.yaw = 0.0  # Face north (0 radians)
                            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                            self.trajectory_setpoint_publisher.publish(msg)
                            
                            # Check if we're close enough to current waypoint
                            distance = euclidean_distance(
                                [self.waypoints[wp_itr][0], self.waypoints[wp_itr][1]], 
                                [self.uav_position[0], self.uav_position[1]]
                            )
                            # If within 0.1m, move to next waypoint
                            if (distance < 0.1):
                                wp_itr = wp_itr + 1
                        else:
                            # All waypoints visited - mission complete
                            mission_done = True
                    else:
                        # Still taking off - hold horizontal position, climb to altitude
                        msg.position = [self.uav_position[0], 
                                      self.uav_position[1], 
                                      z_takeoff]
                        msg.yaw = 0.0  
                        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                        self.trajectory_setpoint_publisher.publish(msg)

                rate.sleep()

    # CALLBACK: maintains offboard control (have to publish at >2Hz)
    def maintain_offboard_cb(self):
        if self.offboard_enable:
            self.send_offboard_signal()

    # CALLBACK: Update armed status from vehicle status
    def status_cb(self, status):
        # arming_state == 2 means ARMED
        self.armed = True if status.arming_state == 2 else False
        
    # CALLBACK: Update offboard mode status
    def vehicle_control_mode_cb(self, mode):
        self.offboard_enable = mode.flag_control_offboard_enabled

    # CALLBACK: Update current position
    def vehicle_local_position_cb(self, local_pos):
        # Store position in NED frame (North, East, Down)
        self.uav_position = [local_pos.x, local_pos.y, local_pos.z]

def euclidean_distance(v1, v2):
    """Calculate 2D distance between two points"""
    return np.linalg.norm(np.array(v1) - np.array(v2))

 
def main(args=None):
    print('Starting UAV Control program...')
    
    rclpy.init(args=args)
    uav_control_node = UAVControl()
    
    # Run main control loop in separate thread so callbacks can still process
    t = threading.Thread(target=uav_control_node.main_loop, args=[])
    t.start()
    
    # Spin to process callbacks (subscribers)
    rclpy.spin(uav_control_node) 
    rclpy.shutdown()


if __name__ == '__main__':
    main()
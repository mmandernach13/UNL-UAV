#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleControlMode
from uav_interfaces.msg import UavPos 
from uav_interfaces.action import GoToPos
import math


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
        self.uav_pos = UavPos()  # Current [x, y, z], yaw position of UAV
        self.take_off = False   # Flag: Has UAV reached takeoff altitude?
        self.offboard_enable = False  # Flag: Is offboard mode active?
        self.armed = False  # Flag: Are motors armed?
        self.action_in_progress = False  # Flag: Is an action currently being executed?
        global offboard_signal_msg
        offboard_signal_msg = OffboardControlMode()

        self.declare_parameter('loop_rate', 10.0)
        self.loop_freq = self.get_parameter('loop_rate').value
        self.rate = self.create_rate(self.loop_freq)

        self.declare_parameter('offboard_rate', 10.0)
        self.offboard_rate = self.get_parameter('offboard_rate').value 

        self.declare_parameter('pos_delta', 0.3)
        self.pos_delta = self.get_parameter('pos_delta').value 

        self.action_server = ActionServer(
            self, 
            GoToPos,
            'go_to_position',
            goal_callback=self.goal_pos_cb,
            cancel_callback=self.cancel_pos_cb,
            execute_callback=self.excecute_pos_cb,
            callback_group=ReentrantCallbackGroup())
        
        self.get_logger().info('Action server started')

        self.maintain_offboard_timer = self.create_timer(1/self.offboard_rate, self.maintain_offboard_cb)
        self.get_logger().info('Offboard maintainer started')

        # SUBSCRIBERS - Listen to PX4 status and position
        # Vehicle position in local North-East-Down (NED) coordinate frame
        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition, 
            '/fmu/out/vehicle_local_position_v1', 
            self.vehicle_local_position_cb, 
            qos_profile
        )
        self.get_logger().info('Position sub created')

        # Vehicle control mode (tells us if offboard mode is active)
        self.vehicle_control_mode_sub = self.create_subscription(
            VehicleControlMode, 
            '/fmu/out/vehicle_control_mode', 
            self.vehicle_control_mode_cb, 
            qos_profile
        )
        self.get_logger().info('Control mode sub created')

        # Vehicle status (arming state, flight mode, etc.)
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, 
            '/fmu/out/vehicle_status_v1', 
            self.status_cb, 
            qos_profile
        )
        self.get_logger().info('Status sub created')

        # PUBLISHERS - Send commands and setpoints to PX4
        # Send high-level commands (arm, disarm, mode changes, land, etc.)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, 
            '/fmu/in/vehicle_command', 
            qos_profile
        )
        self.get_logger().info('Command pub created')

        # Tell PX4 what control mode we want (position, velocity, attitude, etc.)
        # Must publish this at >2Hz or PX4 will exit offboard mode
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, 
            '/fmu/in/offboard_control_mode', 
            qos_profile
        )
        self.get_logger().info('Control mode pub created')

        # Send position/velocity setpoints for the UAV to follow
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, 
            '/fmu/in/trajectory_setpoint', 
            qos_profile
        )
        self.get_logger().info('Trajectory pub created')

    def send_offboard_signal(self):
        offboard_signal_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(offboard_signal_msg)

    def enter_offboard_mode(self):
        # Create offboard control mode message
        # This tells PX4 we want to control POSITION (not velocity/attitude/etc)
        self.get_logger().info('Going into OFFBOARD mode') 
        offboard_signal_msg = OffboardControlMode()
        offboard_signal_msg.position = True  # We'll send position setpoints
        offboard_signal_msg.velocity = False
        offboard_signal_msg.acceleration = False
        offboard_signal_msg.attitude = False
        offboard_signal_msg.body_rate = False

        self.send_offboard_signal()

        itr = 0

        while(self.offboard_enable == False):
            self.get_logger().info('Requesting OFFBOARD mode')
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
        cmd_msg.param1 = 1.0 # 1 to arm, 0 to disarm
        
        max_attempts = 100
        attempts = 0
        
        while (self.armed == False) and (attempts < max_attempts):
            self.get_logger().info(f'Attempting to arm... (attempt {attempts})')

            cmd_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.vehicle_command_publisher.publish(cmd_msg)
           
            attempts += 1
            self.rate.sleep()
        
        if self.armed:
            self.get_logger().info('UAV armed')
        else:
            self.get_logger().error('Failed to arm! Check PX4 console')

    # Every new received goal will be processed here first
    # We can decide to accept or reject the incoming goal
    def goal_pos_cb(self, goal_request: GoToPos.Goal):
        self.get_logger().info('Position goal received')

        new_pos: UavPos = goal_request.target_pos
        if (new_pos.pos[0] == self.uav_pos.pos[0]) and \
           (new_pos.pos[1] == self.uav_pos.pos[1]) and \
           (new_pos.pos[2] == self.uav_pos.pos[2]):
            self.get_logger().info('REJECT already at requested position')
            return GoalResponse.REJECT
        elif new_pos.pos[2] > 0:
            self.get_logger().info('REJECT height request is out of bounds')
            return GoalResponse.REJECT
        # elif new_pos.stamp.nanosec > self.get_clock().now().nanoseconds:
        #     self.get_logger().info('REJECT time stamp of request is in the past')
        #     return GoalResponse.REJECT
        
        self.get_logger().info(f'ACCEPT goal for UavPos: {new_pos.pos}')
        return GoalResponse.ACCEPT 

    # Any cancel request will be processed here, we can accept or reject it
    def cancel_pos_cb(self, goal_handle: ServerGoalHandle):
        self.get_logger().info('Received cancel request')
        # put in hover or land mode
        return CancelResponse.ACCEPT

    # If a goal has been accepted, it will then be executed in this callback
    # After we are done with the goal execution we set a final state and return the result
    # When executing the goal we also check if we need to cancel it
    def excecute_pos_cb(self, goal: ServerGoalHandle):
        self.action_in_progress = True
        self.take_off = False

        self.get_logger().info("Wait for uav initial position...")
        while (len(self.uav_pos.pos) == 0):
             time.sleep(0.1)
        self.get_logger().info("position received")

        try:
            if (self.offboard_enable == False):
                self.enter_offboard_mode()

            if (self.armed == False):
                self.arm_uav()

            target_pos: UavPos = goal.request.target_pos
            result = GoToPos.Result()
            feedback = GoToPos.Feedback()

            self.get_logger().info('Executing Goal')
            dist = self.distance_3d(target_pos.pos, self.uav_pos.pos)
            
            while dist > self.pos_delta:
                if goal.is_cancel_requested:
                    self.get_logger().info('Cancelling goal')
                    goal.canceled()
                    result.success = False 
                    result.message = f'reached position {self.uav_pos}'
                    return result
                
                msg = TrajectorySetpoint()
                        
                # Check if we've reached takeoff altitude
                if(math.fabs(self.uav_pos.pos[2] - target_pos.pos[2]) < self.pos_delta):
                    self.take_off = True
                
                # If at takeoff altitude, start waypoint navigation
                if (self.take_off == True):
                        msg.position = target_pos.pos
                        msg.yaw = target_pos.yaw
                        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                        self.trajectory_setpoint_publisher.publish(msg)
                        
                        # Check if we're close enough to current waypoint
                        distance = self.distance_2d(target_pos.pos, self.uav_pos.pos) 
                else:
                    # Still taking off - hold horizontal position, climb to altitude
                    msg.position = [self.uav_pos.pos[0], 
                                    self.uav_pos.pos[1], 
                                    target_pos.pos[2]]
                    msg.yaw = 0.0  
                    msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                    self.trajectory_setpoint_publisher.publish(msg)
                
                dist = self.distance_3d(target_pos.pos, self.uav_pos.pos)
                cp = UavPos()
                cp = self.uav_pos
                feedback.distance_remaining = dist
                feedback.current_pos = cp
                goal.publish_feedback(feedback)

                self.rate.sleep()
            
            goal.succeed()
            result.success = True
            result.message = f'reached position {self.uav_pos}'
            return result
        
        finally:
            self.action_in_progress = False
   
    # CALLBACK: maintains offboard control (have to publish at >2Hz)
    def maintain_offboard_cb(self):
        if self.offboard_enable:
            self.send_offboard_signal()

            if not self.action_in_progress and self.armed:
                # If no action is in progress, hold current position
                msg = TrajectorySetpoint()
                msg.position = self.uav_pos.pos
                msg.yaw = 0.0
                msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                self.trajectory_setpoint_publisher.publish(msg)

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
        self.uav_pos.pos = [local_pos.x, local_pos.y, local_pos.z]
        self.uav_pos.stamp = local_pos.timestamp

    def distance_2d(self, pos1, pos2):
        # Handle list/tuple input
        x1, y1, _ = pos1[0], pos1[1], pos1[2]
        x2, y2, _ = pos2[0], pos2[1], pos2[2]
        
        dx = x2 - x1
        dy = y2 - y1
        
        return math.sqrt(dx*dx + dy*dy)

    def distance_3d(self, pos1, pos2):
        """
        Calculate 3D Euclidean distance between two positions.
        
        Args:
            pos1: List/tuple of [x, y, z] or object with .x, .y, .z attributes
            pos2: List/tuple of [x, y, z] or object with .x, .y, .z attributes
        
        Returns:
            float: Distance in meters
        """
        # Handle list/tuple input
        x1, y1, z1 = pos1[0], pos1[1], pos1[2]
        x2, y2, z2 = pos2[0], pos2[1], pos2[2]
        
        dx = x2 - x1
        dy = y2 - y1
        dz = z2 - z1
        
        return math.sqrt(dx*dx + dy*dy + dz*dz)

def main(args=None):
    print('Starting UAV Control program...')
    
    rclpy.init(args=args)
    node = PositionController()
    rclpy.spin(node, MultiThreadedExecutor()) 
    rclpy.shutdown()


if __name__ == '__main__':
    main()
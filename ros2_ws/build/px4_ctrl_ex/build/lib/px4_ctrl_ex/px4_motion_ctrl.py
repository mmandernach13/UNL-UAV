import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleControlMode
import threading
import time
import math
import numpy as np

class UAVControl( Node ):
    def __init__(self): 
        super().__init__('uav_control_node')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.uav_position = []
        self.take_off = False
        self.offboard_enable = False
        self.armed = False
        self.vehicle_local_position_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.vehicle_local_position_cb, qos_profile)
        self.vehicle_control_mode_sub = self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self.vehicle_control_mode_cb, qos_profile)
        self.vehicle_status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.status_cb, qos_profile)    
        self.vehicle_command_publisher = self.create_publisher( VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)

        self.waypoints = [[5.0, 0.0], [5.0, 5.0], [0.0, 5.0], [0.0, 0.0]]

   
    def main_loop( self ):

        rate = self.create_rate(10)
        print("Wait for uav initial position...")
        while ( len( self.uav_position) == 0 ):
             time.sleep(0.1)
        print("... position received")

        z_takeoff = self.uav_position[2] - 10.0

        offboard_signal_msg = OffboardControlMode()
        offboard_signal_msg.position = True
        offboard_signal_msg.velocity = False
        offboard_signal_msg.acceleration = False
        offboard_signal_msg.attitude = False
        offboard_signal_msg.body_rate = False

        cmd_msg = VehicleCommand()
        cmd_msg.target_system = 1
        cmd_msg.target_component = 1
        cmd_msg.source_system = 1
        cmd_msg.source_component = 1
        cmd_msg.from_external = True

        wp_itr = 0
        itr = 0
        mission_done = False
        to_exit = False

        while to_exit == False:
        
            offboard_signal_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.offboard_control_mode_publisher.publish(offboard_signal_msg)
            
            if( self.offboard_enable == False ):
                print("Wait offboard mode")
                if( itr > 10 ):
                    cmd_msg = VehicleCommand()
                    cmd_msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
                    cmd_msg.param1 = 1.0
                    cmd_msg.param2 = 6.0 
                    cmd_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                    self.vehicle_command_publisher.publish(cmd_msg)
                itr = itr + 1 
        
            if( self.offboard_enable):
                cmd_msg = VehicleCommand()
                cmd_msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
                cmd_msg.param1 = 1.0 
                cmd_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                self.vehicle_command_publisher.publish(cmd_msg)

            if( mission_done == True ):
                cmd_msg.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
                cmd_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                self.vehicle_command_publisher.publish(cmd_msg)
                to_exit = True
            else:
                if( self.armed ):
                    msg = TrajectorySetpoint()
                    
                    if( math.fabs(self.uav_position[2] - z_takeoff) < 0.3 ):
                        self.take_off = True
                    
                    
                    if ( self.take_off == True ) :
                        if( wp_itr < len(self.waypoints) ):
                            msg.position = [self.waypoints[wp_itr][0], self.waypoints[wp_itr][1], z_takeoff]
                            msg.yaw = 0.0  # (90 degree)
                            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                            self.trajectory_setpoint_publisher.publish(msg)
                            distance = euclidean_distance([self.waypoints[wp_itr][0], self.waypoints[wp_itr][1]], [self.uav_position[0], self.uav_position[1]])
                            if ( distance < 0.1 ):
                                wp_itr = wp_itr+1
                        else:
                            mission_done = True
                    else:
                        msg.position = [self.uav_position[0], self.uav_position[1], z_takeoff]
                        msg.yaw = 0.0  
                        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                        self.trajectory_setpoint_publisher.publish(msg)

                rate.sleep()

    def status_cb( self, status):
        self.armed = True if status.arming_state == 2 else False
        
    def vehicle_control_mode_cb(self, mode ):
        self.offboard_enable = mode.flag_control_offboard_enabled

    def vehicle_local_position_cb(self, local_pos):
        self.uav_position = [local_pos.x, local_pos.y, local_pos.z]

def euclidean_distance(v1, v2):
    return np.linalg.norm(np.array(v1) - np.array(v2))

 
def main(args=None):
    print('Starting UAV Control program...')
    
    rclpy.init(args=args)
    uav_control_node = UAVControl()
    t = threading.Thread(target=uav_control_node.main_loop, args=[])
    t.start()
    rclpy.spin(uav_control_node) 
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# not working, but i dont think it is worth it to get this as it wont be used much

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleControlMode
from pynput import keyboard
import threading
import time

class JoypadSimulator(Node):
    def __init__(self):
        super().__init__('joypad_simulator')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.z_velocity = 0.0
        self.VELOCITY_STEP = 0.05
        self.ANGULAR_VELOCITY_STEP = 0.1

        self.offboard_enable = False
        self.armed = False

        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener.start()
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.vehicle_control_mode_sub = self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self.vehicle_control_mode_cb, qos_profile)
        self.vehicle_status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.status_cb, qos_profile)   
        self.vehicle_command_publisher = self.create_publisher( VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.twist = Twist()

    def on_key_press(self, key):
        try:
            if key.char == 'w':
                self.linear_velocity += self.VELOCITY_STEP
            elif key.char == 's':
                self.linear_velocity -= self.VELOCITY_STEP
            elif key.char == 'a':
                self.angular_velocity += self.ANGULAR_VELOCITY_STEP
            elif key.char == 'd':
                self.angular_velocity -= self.ANGULAR_VELOCITY_STEP
            elif key.char == 'e':
                self.z_velocity += self.VELOCITY_STEP
            elif key.char == 'x':
                self.z_velocity -= self.VELOCITY_STEP
            elif key.char == ' ':
                self.linear_velocity = 0.0
                self.angular_velocity = 0.0
                self.z_velocity = 0.0
            
            self.get_logger().info(f"key press was: {key.char}")
        except AttributeError:
            pass

    def send_cmd_vel(self):
        self.get_logger().info('starting keyboard control')
        rate = self.create_rate(10)  # 10 Hz

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
        
        itr = 0

        while rclpy.ok():
            offboard_signal_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.offboard_control_mode_publisher.publish(offboard_signal_msg)

            if self.offboard_enable == False:
                self.get_logger().info('sent offboard mode cmd')

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
                    
            if(self.offboard_enable and not self.armed):
                cmd_msg = VehicleCommand()
                cmd_msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
                cmd_msg.param1 = 1.0 
                cmd_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                self.vehicle_command_publisher.publish(cmd_msg)
                self.get_logger().info(f'uav is armed: {self.armed}')

            if( self.armed ):
                self.twist.linear.x = self.linear_velocity
                self.twist.linear.z = self.z_velocity
                self.twist.angular.z = self.angular_velocity
                self.vel_pub.publish(self.twist)
                self.get_logger().info(f"publishing twist msg: {self.twist}")
            rate.sleep()

    def status_cb( self, status):
        self.armed = True if status.arming_state == 2 else False
        
    def vehicle_control_mode_cb(self, mode ):
        self.offboard_enable = mode.flag_control_offboard_enabled



def main(args=None):
    rclpy.init(args=args)
    node = JoypadSimulator()
    t = threading.Thread(target=node.send_cmd_vel)
    t.start()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
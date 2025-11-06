import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard
import threading
import subprocess

class JoypadSimulator(Node):
    def __init__(self):
        super().__init__('joypad_simulator')

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.z_velocity = 0.0
        self.VELOCITY_STEP = 0.05
        self.ANGULAR_VELOCITY_STEP = 0.1

        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener.start()
        self.twist = Twist()
        
    def on_key_press(self, key):
        try:
            # Check which key is pressed
            if key.char == 'w':
                self.linear_velocity += self.VELOCITY_STEP
            elif key.char == 'a':
                self.angular_velocity += self.ANGULAR_VELOCITY_STEP
            elif key.char == 'd':
                self.angular_velocity -= self.ANGULAR_VELOCITY_STEP
            elif key.char == 'e':
                self.z_velocity += self.VELOCITY_STEP
            elif key.char == 'x':
                self.z_velocity -= self.VELOCITY_STEP
            elif key.char == 's':
                self.linear_velocity = 0.0
                self.z_velocity = 0.0
                self.angular_velocity = 0.0
        except AttributeError:
            pass

    def send_gazebo_cmd (self):

        rate = self.create_rate(2)
        while rclpy.ok():
            self.twist.linear.x = self.linear_velocity
            self.twist.linear.z = self.z_velocity
            self.twist.angular.z = self.angular_velocity
            command = [
                "ign", "topic", "-t", "/X3/cmd_vel", "-m", "ignition.msgs.Twist",
                "-p", "linear: {x: " + str(self.twist.linear.x) + ", y: 0.0, z: " + str(self.twist.linear.z) + "}, angular: {x: 0.0, y: 0.0, z:" + str(self.twist.angular.z) + "}"
            ]
            self.get_logger().info(f"Publishing: Linear: {self.linear_velocity}, Angular: {self.angular_velocity}")

            subprocess.run(command, capture_output=True, text=True)
            rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    joypad_simulator = JoypadSimulator()
    t = threading.Thread(target=joypad_simulator.send_gazebo_cmd, args=[])
    t.start()
    rclpy.spin(joypad_simulator)
    joypad_simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
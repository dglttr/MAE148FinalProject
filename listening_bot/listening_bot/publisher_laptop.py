from listening_bot.speech_processing import DEFAULT_THROTTLE, DEFAULT_TIMEOUT
from listening_bot.ui import VoiceRecorderUI

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import tkinter as tk

COMMAND_TOPIC_NAME = 'steering_commands'


class SteeringCommandPublisher(Node):

    def __init__(self):
        super().__init__('command_publisher')
        self.publisher_ = self.create_publisher(Twist, COMMAND_TOPIC_NAME, 10)
        self.twist_cmd = Twist()
        
        # Defaults
        self.twist_cmd.linear.x = DEFAULT_THROTTLE
        self.twist_cmd.linear.y = DEFAULT_TIMEOUT
        self.twist_cmd.angular.z = 0.0

        # Launch UI
        self.ui_timer = self.create_timer(0.01, self.launch_ui)

    def launch_ui(self):
        """Launch the User Interface."""
        root = tk.Tk()
        app = VoiceRecorderUI(root, self)
        root.mainloop()

        self.ui_timer.cancel() # cancel timer so it is only executed once

    def publish_new_steering_parameters(self, steering_angle, throttle, timeout):
        """Publish steering parameters (angle, throttle and timeout) to Jetson via topic."""
        if (steering_angle is not None) and (throttle is not None) and (timeout is not None):
            self.get_logger().info(f"Matched steering values: steering angle = {steering_angle}, throttle = {throttle}, timeout = {timeout}")
            self.twist_cmd.angular.z = steering_angle
            self.twist_cmd.linear.x = throttle
            self.twist_cmd.linear.y = timeout
            self.publisher_.publish(self.twist_cmd)


def main(args=None):
    rclpy.init(args=args)

    steering_command_publisher = SteeringCommandPublisher()

    print("Spinning up node")
    rclpy.spin(steering_command_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    steering_command_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
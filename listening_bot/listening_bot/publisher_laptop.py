from speech_to_text import speech_to_text, match_command

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class SteeringCommandPublisher(Node):

    def __init__(self):
        super().__init__('command_publisher')
        self.publisher_ = self.create_publisher(String, 'steering_commands', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()

        text_recognized = speech_to_text(verbose=True)
        intended_command, valid = match_command(text_recognized)
        
        if valid:
            self.get_logger().info(f"Matched command {intended_command}")
            msg.data = intended_command
            self.publisher_.publish(msg)
        else:
            self.get_logger().info("No matching command found")      


def main(args=None):
    rclpy.init(args=args)

    steering_command_publisher = SteeringCommandPublisher()

    rclpy.spin(steering_command_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    steering_command_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
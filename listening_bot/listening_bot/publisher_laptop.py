from listening_bot.speech_processing import speech_to_text, get_steering_values_from_text

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

COMMAND_TOPIC_NAME = 'steering_commands'


class SteeringCommandPublisher(Node):

    def __init__(self):
        super().__init__('command_publisher')
        self.publisher_ = self.create_publisher(Twist, COMMAND_TOPIC_NAME, 10)
        self.twist_cmd = Twist()

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.listen_and_send_command)

    def listen_and_send_command(self):
        """Listen to microphone, find the spoken command, parse steering values and communicate to Jetson."""
        text_recognized = speech_to_text(verbose=True)

        if text_recognized is None:
            return
        
        steering_angle, throttle = get_steering_values_from_text(text_recognized, current_angle=, current_throttle=)
        
        if (steering_angle is not None) and (throttle is not None):
            self.get_logger().info(f"Matched steering values: steering angle = {steering_angle}, throttle = {throttle}")
            self.twist_cmd.angular.z = steering_angle
            self.twist_cmd.linear.x = throttle
            self.publisher_.publish(self.twist_cmd)


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
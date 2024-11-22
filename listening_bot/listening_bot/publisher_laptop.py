from listening_bot.speech_to_text import speech_to_text

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

COMMAND_TOPIC_NAME = 'steering_commands'


def match_command(text_recognized: str) -> tuple[str, bool]:
    """Matches recognized text to the supported commands. Returns command and flag indicating if the command is supported."""
    if text_recognized in ["forward"]:
        return "forward", True
    elif text_recognized in ["backwards", "backward"]:
        return "backward", True
    elif text_recognized in ["left"]:
        return "left", True
    elif text_recognized in ["right"]:
        return "right", True
    elif text_recognized in ["stop"]:
        return "stop", True
    else:
        print(f'No matching command found for recognized text "{text_recognized}"')
        return "", False
    

class SteeringCommandPublisher(Node):

    def __init__(self):
        super().__init__('command_publisher')
        self.publisher_ = self.create_publisher(String, COMMAND_TOPIC_NAME, 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.listen_and_send_command)

    def listen_and_send_command(self):
        """Listen to microphone, find the spoken command and communicate it to Jetson."""
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
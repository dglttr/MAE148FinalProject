from listening_bot.hardware import actuate_forward, actuate_backward, actuate_left, actuate_right, actuate_stop, check_lidar

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


def actuate(command: str):
    if command == "forward":
        actuate_forward()
    elif command == "backward":
        actuate_backward()
    elif command == "left":
        actuate_left()
    elif command == "right":
        actuate_right()
    elif command == "stop":
        actuate_stop()
    else:
        print("Invalid command.")


class SteeringCommandSubscriber(Node):

    def __init__(self):
        super().__init__('steering_command_subscriber')
        self.subscription = self.create_subscription(
            String,
            'steering_commands',
            self.listener_callback,
            10)

        # Regularly check for collisions
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.avoid_collision)
        
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Command received: "{msg.data}". Actuating...')
        #actuate(msg.data)        

    def avoid_collision(self):
        self.get_logger().info('Checking for collisions')
        too_close = check_lidar()

        if too_close:
            actuate("stop")


def main(args=None):
    rclpy.init(args=args)

    steering_command_subscriber = SteeringCommandSubscriber()

    rclpy.spin(steering_command_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    steering_command_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
from listening_bot.hardware import actuate_forward, actuate_backward, actuate_left, actuate_right, actuate_stop, check_lidar

import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

ACTUATOR_TOPIC_NAME = '/cmd_vel'
DEFAULT_THROTTLE = 0.5
MAX_THROTTLE = 1.0
ZERO_THROTTLE = 0.0
MAX_LEFT_ANGLE = -1.0
MAX_RIGHT_ANGLE = 1.0
STRAIGHT_ANGLE = 0.0


def actuate(command: str):
    """Returns tuple (steering_float, throttle_float, valid)"""
    if command == "forward":
        #actuate_forward()
        return STRAIGHT_ANGLE, DEFAULT_THROTTLE, True
    elif command == "backward":
        #actuate_backward()
        return STRAIGHT_ANGLE, -DEFAULT_THROTTLE, True
    elif command == "left":
        #actuate_left()
        return MAX_LEFT_ANGLE, DEFAULT_THROTTLE, True
    elif command == "right":
        #actuate_right()
        return MAX_RIGHT_ANGLE, DEFAULT_THROTTLE, True
    elif command == "stop":
        #actuate_stop()
        return STRAIGHT_ANGLE, ZERO_THROTTLE, True
    else:
        return None, None, False


class SteeringCommandSubscriber(Node):

    def __init__(self):
        super().__init__('steering_command_subscriber')

        # Subscriber
        self.subscription = self.create_subscription(
            String,
            'steering_commands',
            self.listener_callback,
            10)
        
        # Publisher for actuation
        self.twist_publisher = self.create_publisher(Twist, ACTUATOR_TOPIC_NAME, 10)
        self.twist_cmd = Twist()

        # Ensure there are no stops in between
        self.keep_moving_timer = self.create_timer(0.01, self.keep_moving)
        self.current_command = ""

        # Regularly check for collisions
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.avoid_collision)
        
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Command received: "{msg.data}". Actuating...')
        
        steering_float, throttle_float, valid = actuate(msg.data)

        if valid:
            try:
                # publish control signals
                self.twist_cmd.angular.z = steering_float
                self.twist_cmd.linear.x = throttle_float
                self.twist_publisher.publish(self.twist_cmd)
            except KeyboardInterrupt:
                self.twist_cmd.linear.x = ZERO_THROTTLE     # stop movement
                self.twist_publisher.publish(self.twist_cmd)

    def keep_moving(self):
        self.twist_publisher.publish(self.twist_cmd)
        
    def avoid_collision(self):
        self.get_logger().info('Checking for collisions')
        too_close = check_lidar()

        if too_close:
            actuate("stop")


def testing():
    publisher = Node.create_publisher(String, 'steering_commands', 10)
    msg = String()

    msg.data = "forward"
    publisher.publish(msg)
    time.sleep(3)

    msg.data = "backward"
    publisher.publish(msg)
    time.sleep(3)

    msg.data = "left"
    publisher.publish(msg)
    time.sleep(3)

    msg.data = "right"
    publisher.publish(msg)
    time.sleep(3)

    msg.data = "stop"
    publisher.publish(msg)


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
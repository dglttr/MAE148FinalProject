import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

COMMAND_TOPIC_NAME = 'steering_commands'
ACTUATOR_TOPIC_NAME = '/cmd_vel'
LIDAR_TOPIC_NAME = 'scan'

DEFAULT_THROTTLE = 0.5
MAX_THROTTLE = 1.0
ZERO_THROTTLE = 0.0
MAX_LEFT_ANGLE = -1.0
MAX_RIGHT_ANGLE = 1.0
STRAIGHT_ANGLE = 0.0


def get_steering_values_from_command(command: str) -> tuple[float, float, bool]:
    """Translates passed command into steering angle and throttle values.
    
    :Returns: tuple (steering_float, throttle_float, valid)
    """
    if command == "forward":
        return STRAIGHT_ANGLE, DEFAULT_THROTTLE, True
    elif command == "backward":
        return STRAIGHT_ANGLE, -DEFAULT_THROTTLE, True
    elif command == "left":
        return MAX_LEFT_ANGLE, DEFAULT_THROTTLE, True
    elif command == "right":
        return MAX_RIGHT_ANGLE, DEFAULT_THROTTLE, True
    elif command == "stop":
        return STRAIGHT_ANGLE, ZERO_THROTTLE, True
    else:
        return None, None, False


class SteeringCommandSubscriber(Node):

    def __init__(self):
        super().__init__('steering_command_subscriber')

        # Commands subscription
        self.commands_subscription = self.create_subscription(String, COMMAND_TOPIC_NAME, self.command_callback, 10)

        # TODO LIDAR subscription
        self.lidar_subscription = self.create_subscription(LaserScan, LIDAR_TOPIC_NAME, self.lidar_callback, 10)
        
        # Publisher for actuation
        self.twist_publisher = self.create_publisher(Twist, ACTUATOR_TOPIC_NAME, 10)
        self.twist_cmd = Twist()

        # Ensure car keeps moving until receiving another command
        self.keep_moving_timer = self.create_timer(0.01, self.keep_moving)
        self.current_command = ""

    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Command received: "{command}". Actuating...')
        
        steering_float, throttle_float, valid = get_steering_values_from_command(command)

        if valid:
            try:
                self.publish_to_vesc(steering_float, throttle_float)
            except KeyboardInterrupt:
                self.publish_to_vesc(STRAIGHT_ANGLE, ZERO_THROTTLE)     # stop movement

    def keep_moving(self):
        self.twist_publisher.publish(self.twist_cmd)
        
    def lidar_callback(self, msg):
        too_close = False#msg.data    # TODO check array

        if too_close:
            self.publish_to_vesc(STRAIGHT_ANGLE, ZERO_THROTTLE)     # stop movement

    def publish_to_vesc(self, steering_angle: float, throttle: float):
        self.twist_cmd.angular.z = steering_angle
        self.twist_cmd.linear.x = throttle
        self.twist_publisher.publish(self.twist_cmd)


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
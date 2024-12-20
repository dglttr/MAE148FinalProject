import os
from datetime import datetime

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from roboflowoak import RoboflowOak

COMMAND_TOPIC_NAME = 'steering_commands'
ACTUATOR_TOPIC_NAME = '/cmd_vel'
LIDAR_TOPIC_NAME = '/scan'

ZERO_THROTTLE = 0.0
STRAIGHT_ANGLE = 0.0

MIN_ALLOWED_DISTANCE = 1.0  # when LIDAR detects something at a closer distance, it stops the car [in meters] 

STOP_SIGN_DETECTION_CONFIDENCE = 0.85    # how confident the model should be in detecting a stop sign [0-1]
STOP_SIGN_DETECTION_DISTANCE = 1    # how far the stop sign would be away [meters]


class SteeringCommandSubscriber(Node):

    def __init__(self):
        super().__init__('steering_command_subscriber')

        # Commands subscription
        self.commands_subscription = self.create_subscription(Twist, COMMAND_TOPIC_NAME, self.command_callback, 10)

        # LIDAR subscription
        self.lidar_subscription = self.create_subscription(LaserScan, LIDAR_TOPIC_NAME, self.lidar_callback, 10)
        
        # Publisher for actuation
        self.twist_publisher = self.create_publisher(Twist, ACTUATOR_TOPIC_NAME, 10)
        self.twist_cmd = Twist()

        # Ensure car keeps moving until receiving another command
        self.keep_moving_timer = self.create_timer(0.01, self.keep_moving)
        self.command_start_time = None
        self.timeout = 10    # when to stop after receiving command (in seconds)

        # Use model on OAK-D for stop sign detection
        try:
            self.stop_sign_detection_model = RoboflowOak(model="stopsigndetection-xuceg", confidence=STOP_SIGN_DETECTION_CONFIDENCE,
                                                         overlap=0.5, version="1", api_key=os.environ['ROBOFLOW_API_KEY'],
                                                         rgb=True, depth=True, device=None, blocking=True)
            self.stop_sign_detection_timer = self.create_timer(0.01, self.stop_at_stop_sign)
        except Exception as e:
            self.get_logger().warning(f'Could not create stop sign detection model due to: {e}. Will NOT check for stop signs.')

        self.get_logger().info('Start listening for commands...')

    def command_callback(self, msg):
        """Move car according to incoming commands."""
        steering_angle = msg.angular.z
        throttle = msg.linear.x
        timeout = msg.linear.y
        self.get_logger().info(f'Received steering values: steering angle = {steering_angle}, throttle = {throttle}, timeout = {timeout}. Actuating...')

        try:
            self.command_start_time = datetime.now()
            self.timeout = timeout
            self.publish_to_vesc(steering_angle, throttle)
        except KeyboardInterrupt:
            self.stop_car()

    def stop_car(self):
        self.publish_to_vesc(STRAIGHT_ANGLE, ZERO_THROTTLE)
    
    def keep_moving(self):
        """Keep moving according to last command unless more than X seconds have passed (timeout)."""
        if self.command_start_time is None:    # at the beginning, when no command has been sent yet
            return        
        
        time_passed = (datetime.now() - self.command_start_time).seconds
        if time_passed <= self.timeout:      # Check that timeout has not lapsed
            self.twist_publisher.publish(self.twist_cmd)
        else:
            self.get_logger().info(f"Command timed out after {self.timeout} seconds. Stopping car...")
            self.stop_car()
            self.command_start_time = None  # reset safety timeout
        
    def lidar_callback(self, msg):
        """Listen to LIDAR signal and stop car if an obstacle gets too close."""
        ranges = msg.ranges
        # Filter: Only check between 10% and 40% (roughly front third)
        start_idx = int(len(ranges) * 0.1)
        end_idx = int(len(ranges) * 0.4)
        front_ranges = ranges[start_idx:end_idx]
        min_distance = min(front_ranges)

        if (min_distance <= MIN_ALLOWED_DISTANCE) and (min_distance >= 0.1):    # also check that min distance is greater than 0.1m to avoid triggers from erroneous readings or objects that are on the car itself
            self.get_logger().info(f'Too close ({min_distance} m). Stopping vehicle...')
            self.stop_car()

    def publish_to_vesc(self, steering_angle: float, throttle: float):
        """Publish steering angle and throttle value to VESC topic (/cmd_vel),"""
        self.twist_cmd.angular.z = steering_angle
        self.twist_cmd.linear.x = throttle
        self.twist_publisher.publish(self.twist_cmd)

    def stop_at_stop_sign(self):
        """Try to detect a stop sign using OAK-D camera and stop when it is too close."""
        result, frame, raw_frame, depth = self.stop_sign_detection_model.detect()
        for prediction in result["predictions"]:
            if prediction.depth <= STOP_SIGN_DETECTION_DISTANCE:
                #max_depth = np.amax(depth) -- need to normalize depths? 
                self.get_logger().info(f'Stop sign detected at {prediction.depth} m distance with {prediction.confidence*100:.1f}% confidence. Stopping car...')
                self.stop_car()
                return


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
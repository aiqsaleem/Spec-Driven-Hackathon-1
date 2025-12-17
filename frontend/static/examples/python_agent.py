#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math
import random

class SimpleAIAgent(Node):
    def __init__(self):
        super().__init__('simple_ai_agent')

        # Subscribe to laser scan data
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publisher for robot commands
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Publisher for AI state (for monitoring)
        self.state_publisher = self.create_publisher(Float32, 'ai_state', 10)

        # Timer for AI processing loop
        self.timer = self.create_timer(0.2, self.ai_processing_loop)

        # Store latest sensor data
        self.latest_scan = None

        # AI state variables
        self.ai_state = 0.0  # 0.0 = exploring, 1.0 = avoiding obstacle

    def laser_callback(self, msg):
        # Store the latest laser scan data
        self.latest_scan = msg
        self.get_logger().info(f'Received laser scan with {len(msg.ranges)} ranges')

    def ai_processing_loop(self):
        if self.latest_scan is not None:
            # Process sensor data through AI algorithm
            command = self.simple_navigation_ai()

            # Publish the command to the robot
            self.cmd_publisher.publish(command)

            # Publish AI state for monitoring
            state_msg = Float32()
            state_msg.data = self.ai_state
            self.state_publisher.publish(state_msg)

    def simple_navigation_ai(self):
        """
        Simple AI algorithm:
        - If no obstacles within 1 meter ahead, move forward
        - If obstacles detected, turn to avoid
        """
        if self.latest_scan is None:
            # If no scan data, stop the robot
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            return cmd

        # Get the front-facing range readings (center 30 degrees)
        num_ranges = len(self.latest_scan.ranges)
        front_start = num_ranges // 2 - 15
        front_end = num_ranges // 2 + 15

        # Make sure indices are within bounds
        front_start = max(0, front_start)
        front_end = min(num_ranges, front_end)

        front_ranges = self.latest_scan.ranges[front_start:front_end]

        # Remove invalid range values (inf, nan) and get minimum distance
        valid_ranges = [r for r in front_ranges if not (math.isinf(r) or math.isnan(r)) and r > 0]

        if not valid_ranges:
            # If no valid ranges, stop
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.ai_state = 0.0
            return cmd

        min_distance = min(valid_ranges)

        cmd = Twist()

        if min_distance > 1.0:
            # Move forward if no obstacles within 1 meter
            cmd.linear.x = 0.3  # Move forward at 0.3 m/s
            cmd.angular.z = 0.0
            self.ai_state = 0.0  # Exploring state
        else:
            # Turn to avoid obstacle
            cmd.linear.x = 0.0
            cmd.angular.z = 0.8  # Turn at 0.8 rad/s
            self.ai_state = 1.0  # Avoiding obstacle state

        return cmd

def main(args=None):
    rclpy.init(args=args)
    ai_agent = SimpleAIAgent()

    try:
        rclpy.spin(ai_agent)
    except KeyboardInterrupt:
        pass
    finally:
        ai_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
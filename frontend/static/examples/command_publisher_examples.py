#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math
import time

class CommandPublishingExamples(Node):
    def __init__(self):
        super().__init__('command_publishing_examples')

        # Publisher for robot velocity commands
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Example 1: Publisher with rate limiting
        self.rate_limited_publisher = self.create_publisher(Twist, 'cmd_vel_rate_limited', 10)
        self.command_rate = 10  # Hz
        self.last_command_time = self.get_clock().now()

        # Example 2: Publisher with safety constraints
        self.safe_publisher = self.create_publisher(Twist, 'cmd_vel_safe', 10)

        # Example 3: Publisher with emergency stop
        self.emergency_stop_publisher = self.create_publisher(Twist, 'cmd_vel_emergency', 10)
        self.emergency_stop_active = False

        # Subscriber for sensor data to demonstrate conditional publishing
        self.scan_subscription = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)

        # Timer to demonstrate different publishing strategies
        self.example_timer = self.create_timer(0.5, self.run_examples)

        self.latest_scan = None
        self.example_counter = 0

    def scan_callback(self, msg):
        self.latest_scan = msg

    def run_examples(self):
        self.example_counter += 1

        if self.example_counter % 4 == 1:
            self.example_basic_publishing()
        elif self.example_counter % 4 == 2:
            self.example_rate_limited_publishing()
        elif self.example_counter % 4 == 3:
            self.example_safe_publishing()
        else:
            self.example_conditional_publishing()

    def example_basic_publishing(self):
        """Basic command publishing example"""
        cmd = Twist()
        cmd.linear.x = 0.5  # Move forward at 0.5 m/s
        cmd.angular.z = 0.0

        self.cmd_publisher.publish(cmd)
        self.get_logger().info("Basic publishing: Moving forward")

    def example_rate_limited_publishing(self):
        """Rate-limited command publishing example"""
        current_time = self.get_clock().now()

        # Only publish if enough time has passed
        if (current_time - self.last_command_time).nanoseconds >= 1e9 / self.command_rate:
            cmd = Twist()
            cmd.linear.x = 0.3
            cmd.angular.z = 0.5  # Turn while moving

            self.rate_limited_publisher.publish(cmd)
            self.last_command_time = current_time
            self.get_logger().info("Rate-limited publishing: Turn while moving")

    def example_safe_publishing(self):
        """Safe command publishing with constraints"""
        cmd = Twist()
        cmd.linear.x = 2.0  # This would be too fast
        cmd.angular.z = 3.0  # This would be too fast

        # Apply safety constraints
        cmd = self.apply_safety_constraints(cmd)

        self.safe_publisher.publish(cmd)
        self.get_logger().info(f"Safe publishing: linear.x={cmd.linear.x}, angular.z={cmd.angular.z}")

    def example_conditional_publishing(self):
        """Conditional publishing based on sensor data"""
        if self.latest_scan is not None:
            # Check if there's an obstacle in front
            front_ranges = self.latest_scan.ranges[len(self.latest_scan.ranges)//2-10:len(self.latest_scan.ranges)//2+10]
            min_front_distance = min([r for r in front_ranges if not (math.isinf(r) or math.isnan(r)) and r > 0] or [float('inf')])

            cmd = Twist()
            if min_front_distance > 1.0:
                cmd.linear.x = 0.4  # Move forward if no obstacles
                cmd.angular.z = 0.0
                self.get_logger().info("Conditional publishing: Moving forward (no obstacles)")
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.6  # Turn to avoid obstacle
                self.get_logger().info("Conditional publishing: Turning to avoid obstacle")

            self.emergency_stop_publisher.publish(cmd)

    def apply_safety_constraints(self, cmd):
        """Apply safety constraints to the command"""
        # Limit maximum velocities
        cmd.linear.x = max(min(cmd.linear.x, 0.5), -0.5)  # Max ±0.5 m/s
        cmd.angular.z = max(min(cmd.angular.z, 1.0), -1.0)  # Max ±1.0 rad/s

        # Emergency stop if needed
        if self.emergency_stop_active:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        return cmd

    def activate_emergency_stop(self):
        """Activate emergency stop"""
        self.emergency_stop_active = True
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.emergency_stop_publisher.publish(cmd)
        self.get_logger().info("Emergency stop activated")

def main(args=None):
    rclpy.init(args=args)
    examples = CommandPublishingExamples()

    try:
        rclpy.spin(examples)
    except KeyboardInterrupt:
        pass
    finally:
        examples.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
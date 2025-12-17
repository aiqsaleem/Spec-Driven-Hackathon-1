#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, BatteryState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math

class ControllerBridgeNode(Node):
    def __init__(self):
        super().__init__('controller_bridge')

        # Multiple subscriptions for different sensor data
        self.laser_subscription = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10)

        self.odom_subscription = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)

        self.battery_subscription = self.create_subscription(
            BatteryState, 'battery_status', self.battery_callback, 10)

        # Publisher for robot commands
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Publisher for processed sensor data (for AI agents)
        self.processed_data_publisher = self.create_publisher(Float32, 'processed_sensor_data', 10)

        # Store robot state
        self.robot_state = {
            'position': None,
            'orientation': None,
            'battery_level': 100.0,
            'obstacle_distances': [],
            'linear_velocity': 0.0,
            'angular_velocity': 0.0
        }

        # AI processing timer
        self.ai_timer = self.create_timer(0.1, self.process_and_publish_state)

        # Command rate limiting
        self.command_rate = 10  # Hz
        self.last_command_time = self.get_clock().now()

    def laser_callback(self, msg):
        self.robot_state['obstacle_distances'] = msg.ranges

    def odom_callback(self, msg):
        self.robot_state['position'] = msg.pose.pose.position
        self.robot_state['orientation'] = msg.pose.pose.orientation
        self.robot_state['linear_velocity'] = msg.twist.twist.linear.x
        self.robot_state['angular_velocity'] = msg.twist.twist.angular.z

    def battery_callback(self, msg):
        self.robot_state['battery_level'] = msg.percentage

    def process_and_publish_state(self):
        # Process sensor data for AI agents
        processed_data = self.calculate_safety_metrics()

        # Publish processed data for AI agents to consume
        data_msg = Float32()
        data_msg.data = processed_data
        self.processed_data_publisher.publish(data_msg)

    def calculate_safety_metrics(self):
        """
        Calculate safety metrics from sensor data
        Returns a value between 0.0 (unsafe) and 1.0 (safe)
        """
        if not self.robot_state['obstacle_distances']:
            return 0.5  # Neutral if no data

        # Calculate minimum distance to obstacles
        valid_distances = [d for d in self.robot_state['obstacle_distances']
                          if not (math.isinf(d) or math.isnan(d)) and d > 0]

        if not valid_distances:
            return 0.5  # Neutral if no valid distances

        min_distance = min(valid_distances)

        # Normalize to 0-1 scale (safe at 1m+ distance)
        safety_score = min(1.0, min_distance)

        # Factor in battery level (lower battery = less safe for long operations)
        battery_factor = self.robot_state['battery_level'] / 100.0

        # Combined safety score
        combined_score = (safety_score + battery_factor) / 2.0

        return combined_score

    def send_command(self, cmd):
        """
        Send command with safety checks and rate limiting
        """
        current_time = self.get_clock().now()

        # Rate limiting: only send command if enough time has passed
        if (current_time - self.last_command_time).nanoseconds < 1e9 / self.command_rate:
            return  # Skip if too soon

        # Apply safety constraints
        cmd = self.apply_safety_constraints(cmd)

        # Publish the command
        self.cmd_publisher.publish(cmd)
        self.last_command_time = current_time

    def apply_safety_constraints(self, cmd):
        """
        Apply safety constraints to the command
        """
        # Limit maximum velocities
        cmd.linear.x = max(min(cmd.linear.x, 0.5), -0.5)  # Max ±0.5 m/s
        cmd.angular.z = max(min(cmd.angular.z, 1.0), -1.0)  # Max ±1.0 rad/s

        # Emergency stop if battery is critically low
        if self.robot_state['battery_level'] < 5.0:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # Stop if too close to obstacles
        if self.robot_state['obstacle_distances']:
            valid_distances = [d for d in self.robot_state['obstacle_distances']
                              if not (math.isinf(d) or math.isnan(d)) and d > 0 and d < 0.3]
            if valid_distances:  # If obstacles within 0.3m
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

        return cmd

def main(args=None):
    rclpy.init(args=args)
    bridge = ControllerBridgeNode()

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
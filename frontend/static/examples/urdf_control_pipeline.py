#!/usr/bin/env python3
"""
URDF-to-ROS2 Control Pipeline Example

This script demonstrates how to connect a URDF model to ROS 2 control systems
using the ros2_control framework. It shows the integration between robot models
and control interfaces.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import time


class URDFControlPipeline(Node):
    """
    Example node demonstrating the connection between URDF models and ROS 2 control systems.
    This shows how to interface with ros2_control framework for hardware control.
    """

    def __init__(self):
        super().__init__('urdf_control_pipeline')

        # Joint state publisher for robot state visualization
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Command publisher for joint trajectory control
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10
        )

        # State subscriber to monitor current joint positions
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )

        # Timer for publishing joint states
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        # Timer for sending trajectory commands (every 2 seconds)
        self.trajectory_timer = self.create_timer(2.0, self.send_trajectory_command)

        # Store current joint positions
        self.current_positions = {}

        # Define joint names that match our URDF model
        self.joint_names = [
            'shoulder_abduction_joint',
            'shoulder_flexion_joint',
            'elbow_joint',
            'wrist_joint'
        ]

        self.get_logger().info('URDF Control Pipeline node initialized')

    def joint_state_callback(self, msg):
        """Callback to update current joint positions"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_positions[name] = msg.position[i]

    def publish_joint_states(self):
        """Publish current joint states for robot visualization"""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []

        # Generate example joint positions (oscillating motion)
        for i, joint_name in enumerate(self.joint_names):
            # Create oscillating motion for demonstration
            position = math.sin(self.get_clock().now().nanoseconds / 1e9 + i) * (math.pi / 4)
            msg.name.append(joint_name)
            msg.position.append(position)
            msg.velocity.append(0.0)  # Placeholder
            msg.effort.append(0.0)    # Placeholder

        self.joint_state_pub.publish(msg)

    def send_trajectory_command(self):
        """Send a trajectory command to the joint trajectory controller"""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        # Create trajectory point
        point = JointTrajectoryPoint()

        # Set target positions (creating a wave-like motion)
        current_time = self.get_clock().now().nanoseconds / 1e9
        point.positions = [
            math.sin(current_time) * 0.5,           # shoulder abduction
            math.sin(current_time + 0.5) * 0.5,     # shoulder flexion
            math.sin(current_time + 1.0) * 1.0,     # elbow
            math.sin(current_time + 1.5) * 0.3      # wrist
        ]

        # Set velocities
        point.velocities = [0.0] * len(self.joint_names)

        # Set accelerations
        point.accelerations = [0.0] * len(self.joint_names)

        # Set effort
        point.effort = [0.0] * len(self.joint_names)

        # Set time from start (1 second duration)
        point.time_from_start = Duration(sec=1, nanosec=0)

        msg.points = [point]

        self.joint_trajectory_pub.publish(msg)
        self.get_logger().info(f'Sent trajectory command: {point.positions}')


class RobotStatePublisher(Node):
    """
    Simpler example of robot state publisher that takes URDF and publishes transforms
    """

    def __init__(self):
        super().__init__('robot_state_publisher_example')

        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Timer to publish joint states
        self.timer = self.create_timer(0.05, self.publish_joint_states)

        # Joint names from URDF
        self.joint_names = ['joint_name']  # This would match your URDF joint names

        self.get_logger().info('Robot State Publisher example initialized')

    def publish_joint_states(self):
        """Publish joint states based on current robot configuration"""
        msg = JointState()
        msg.name = self.joint_names
        msg.position = [math.sin(self.get_clock().now().nanoseconds / 1e9)]  # Example position
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        self.joint_pub.publish(msg)


class JointStatePublisher(Node):
    """
    Example of joint state publisher with GUI-like functionality
    """

    def __init__(self):
        super().__init__('joint_state_publisher_example')

        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        # Simulated joint values (would come from GUI sliders in real implementation)
        self.joint_values = {
            'shoulder_abduction_joint': 0.0,
            'shoulder_flexion_joint': 0.0,
            'elbow_joint': 0.0,
            'wrist_joint': 0.0
        }

        self.get_logger().info('Joint State Publisher example initialized')

    def publish_joint_states(self):
        """Publish joint states with simulated values"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Update joint values with oscillating motion for demonstration
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.joint_values['shoulder_abduction_joint'] = math.sin(current_time) * 0.5
        self.joint_values['shoulder_flexion_joint'] = math.sin(current_time + 1.0) * 0.5
        self.joint_values['elbow_joint'] = math.sin(current_time + 0.5) * 1.0
        self.joint_values['wrist_joint'] = math.sin(current_time + 1.5) * 0.3

        msg.name = list(self.joint_values.keys())
        msg.position = list(self.joint_values.values())
        msg.velocity = [0.0] * len(msg.position)
        msg.effort = [0.0] * len(msg.position)

        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    # You can run any of these examples:
    # node = RobotStatePublisher()  # Basic robot state publisher
    # node = JointStatePublisher()  # Joint state publisher with simulated values
    node = URDFControlPipeline()   # Full control pipeline example

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
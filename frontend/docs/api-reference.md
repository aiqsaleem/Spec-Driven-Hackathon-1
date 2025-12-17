---
sidebar_position: 101
---

# API Reference

This page provides references to key ROS 2 APIs used throughout the course.

## rclpy API

### Node Class
- `rclpy.node.Node` - Base class for creating ROS 2 nodes
- `create_publisher()` - Create a publisher for a topic
- `create_subscription()` - Create a subscriber for a topic
- `create_service()` - Create a service server
- `create_client()` - Create a service client
- `create_timer()` - Create a timer for periodic callbacks

### Core Functions
- `rclpy.init()` - Initialize the ROS 2 client library
- `rclpy.spin()` - Process callbacks until shutdown
- `rclpy.shutdown()` - Shutdown the ROS 2 client library

## Message Types

### Common Message Types
- `std_msgs.msg` - Standard message types (String, Int32, Float64, etc.)
- `sensor_msgs.msg` - Sensor data messages (JointState, Image, LaserScan, etc.)
- `geometry_msgs.msg` - Geometric messages (Point, Pose, Twist, etc.)
- `nav_msgs.msg` - Navigation messages (Odometry, Path, etc.)

## Quality of Service (QoS) Profiles

### Built-in Profiles
- `QoSProfile(depth=10)` - Default profile with history depth
- `ReliabilityPolicy.RELIABLE` - Reliable delivery
- `ReliabilityPolicy.BEST_EFFORT` - Best effort delivery
- `DurabilityPolicy.VOLATILE` - Volatile durability
- `DurabilityPolicy.TRANSIENT_LOCAL` - Transient local durability

## TF2 API

### Transform Operations
- `tf2_ros.TransformBroadcaster` - Broadcast transforms
- `tf2_ros.TransformListener` - Listen to transforms
- `tf2_ros.Buffer` - Store and lookup transforms

## ros2_control API

### Controller Interfaces
- `JointTrajectoryController` - Control joint trajectories
- `ForwardCommandController` - Forward commands directly
- `JointStateBroadcaster` - Broadcast joint states

## Common Parameter Types

### Joint State Messages
- `JointState.name` - Array of joint names
- `JointState.position` - Array of joint positions
- `JointState.velocity` - Array of joint velocities
- `JointState.effort` - Array of joint efforts

## URDF-Related Tools

### Command Line Tools
- `check_urdf` - Validate URDF files
- `xacro` - Process xacro files to URDF
- `ros2 run robot_state_publisher` - Publish robot state transforms
- `ros2 run joint_state_publisher` - Publish joint state messages
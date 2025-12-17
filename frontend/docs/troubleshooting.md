---
sidebar_position: 102
---

# Troubleshooting Guide

This guide addresses common issues encountered when working with ROS 2 and the concepts covered in this course.

## Common Installation Issues

### ROS 2 Installation
- **Issue**: `ros2` command not found
  - **Solution**: Check that ROS 2 is installed and sourced properly
  - **Command**: `source /opt/ros/humble/setup.bash` (or your ROS 2 distribution)

- **Issue**: Python packages not found
  - **Solution**: Ensure you're using the correct Python environment
  - **Command**: `python3 -c "import rclpy"` to test installation

### Dashing/Performance Issues
- **Issue**: Slow communication between nodes
  - **Solution**: Check network configuration and QoS settings
  - **Check**: Ensure nodes are on the same network domain

## URDF Validation Issues

### XML Syntax Errors
- **Issue**: URDF file fails to load
  - **Solution**: Validate with `check_urdf /path/to/robot.urdf`
  - **Common cause**: Unclosed tags or invalid XML syntax

### Joint and Link Errors
- **Issue**: Missing parent/child links in URDF
  - **Solution**: Verify all joint parent and child names match link names
  - **Check**: Ensure the kinematic chain is complete from base to end-effector

### Inertial Properties
- **Issue**: Simulation instability
  - **Solution**: Ensure all links have proper mass and inertia values
  - **Note**: Use realistic values for stable simulation

## rclpy Development Issues

### Node Communication
- **Issue**: Nodes can't communicate across topics
  - **Solution**: Check topic names match exactly (case-sensitive)
  - **Check**: Verify QoS profiles are compatible between publisher/subscriber

### Import Errors
- **Issue**: `ModuleNotFoundError: No module named 'rclpy'`
  - **Solution**: Source ROS 2 environment and check Python path
  - **Command**: `echo $PYTHONPATH` to verify paths

### Lifecycle Issues
- **Issue**: Node stops unexpectedly
  - **Solution**: Implement proper exception handling
  - **Check**: Ensure all resources are properly cleaned up

## Visualization Issues

### RViz2 Problems
- **Issue**: Robot model doesn't appear in RViz2
  - **Solution**: Check that robot_description parameter is set correctly
  - **Check**: Verify joint_states topic is being published

- **Issue**: TF transforms not showing
  - **Solution**: Ensure robot_state_publisher is running
  - **Command**: `ros2 run robot_state_publisher robot_state_publisher`

### Joint State Visualization
- **Issue**: Joints not moving in visualization
  - **Solution**: Verify joint names match between URDF and joint state messages
  - **Check**: Ensure joint positions are being published correctly

## Control System Issues

### ros2_control Problems
- **Issue**: Controllers not loading
  - **Solution**: Check controller configuration files and launch order
  - **Check**: Verify controller manager is running before loading controllers

- **Issue**: Command interfaces not working
  - **Solution**: Ensure command interfaces match controller configuration
  - **Check**: Verify hardware interface compatibility

## Network and Multi-Device Issues

### DDS Communication
- **Issue**: Nodes on different machines can't communicate
  - **Solution**: Check firewall settings and RMW configuration
  - **Check**: Ensure machines are on the same network and discovery is working

- **Issue**: High latency in communication
  - **Solution**: Optimize QoS settings for your use case
  - **Consider**: Using reliable vs best-effort transports appropriately

## Performance Optimization

### Memory Usage
- **Issue**: High memory consumption
  - **Solution**: Reduce message queue sizes and update rates
  - **Check**: Monitor for memory leaks in long-running nodes

### CPU Usage
- **Issue**: High CPU usage
  - **Solution**: Optimize timer frequencies and callback execution
  - **Consider**: Using threading for computationally expensive operations

## Debugging Strategies

### Using ROS 2 Tools
- **Command**: `ros2 topic list` - Check available topics
- **Command**: `ros2 node list` - Check running nodes
- **Command**: `ros2 topic echo <topic_name>` - Monitor topic data
- **Command**: `ros2 service list` - Check available services

### Logging
- Use `self.get_logger().info()` for debugging information
- Use appropriate log levels (debug, info, warn, error, fatal)
- Check log files in `~/.ros/log/` for detailed error information

### Testing
- Test components individually before integration
- Use simulation before testing on real hardware
- Implement unit tests for critical functions
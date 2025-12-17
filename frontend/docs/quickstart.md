---
sidebar_position: 2
---

# Quickstart Guide

Get started with the ROS 2 Fundamentals course in minutes.

## Prerequisites

Before starting this course, ensure you have:

- ROS 2 Humble Hawksbill installed (or your preferred ROS 2 distribution)
- Python 3.8 or higher
- Basic Python programming knowledge
- Familiarity with command-line tools

## Installation and Setup

### 1. Install ROS 2

Follow the official ROS 2 installation guide for your platform:
```bash
# Ubuntu/Debian
sudo apt update && sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
```

### 2. Install Additional Dependencies

```bash
# Install Python dependencies
pip3 install rclpy

# Install URDF tools
sudo apt install ros-humble-urdfdom-tools ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui
```

### 3. Clone or Access the Course Materials

All examples and exercises for this course are available in the `static/examples/` directory of the documentation site.

## Course Structure

This course is organized into three main chapters:

### Chapter 1: ROS 2 Communication Fundamentals
- Learn about nodes, topics, services, and actions
- Understand Quality of Service (QoS) settings
- Practice with simple publisher/subscriber examples

### Chapter 2: Python Agents to Robot Control
- Interface Python-based AI agents with ROS controllers
- Use rclpy to create ROS 2 nodes in Python
- Implement AI-to-robot communication patterns

### Chapter 3: Robot Body Modeling with URDF
- Model robot bodies using URDF (Unified Robot Description Format)
- Add sensors to robot models
- Connect URDF models to ROS 2 control pipelines

## Getting Started with Examples

### Running a Simple Publisher

1. Navigate to the examples directory
2. Run the simple publisher example:
```bash
cd docs-site/my-website/static/examples
python3 simple_publisher.py
```

### Running a Simple Subscriber

In another terminal:
```bash
cd docs-site/my-website/static/examples
python3 simple_subscriber.py
```

### Visualizing a Robot Model

1. Launch the robot state publisher:
```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:='$(cat basic_robot.urdf)'
```

2. Launch RViz2 to visualize:
```bash
rviz2
```

## Development Environment

### Using the Docusaurus Site

The course content is served using Docusaurus. To run locally:

1. Install Node.js (v16 or higher)
2. Navigate to the docs-site directory:
```bash
cd docs-site/my-website
```

3. Install dependencies:
```bash
npm install
```

4. Start the development server:
```bash
npm start
```

### Example Files

All example files are located in `docs-site/my-website/static/examples/` and include:

- Python scripts for ROS 2 nodes
- URDF files for robot models
- Configuration files for controllers
- Launch files for complete systems

## Next Steps

After completing this quickstart:

1. Read [Chapter 1: ROS 2 Communication Fundamentals](./module-1/chapter-1.md) to understand basic ROS 2 concepts
2. Try the hands-on exercises in each chapter
3. Experiment with the provided examples
4. Build your own ROS 2 applications using the patterns learned

## Troubleshooting

If you encounter issues:

1. Check the [Troubleshooting Guide](./troubleshooting.md) for common problems
2. Verify your ROS 2 installation with `ros2 topic list`
3. Ensure proper environment setup with `source /opt/ros/humble/setup.bash`
4. Confirm all dependencies are installed
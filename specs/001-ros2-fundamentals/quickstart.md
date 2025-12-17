# Quickstart Guide: ROS 2 Fundamentals Course

## Prerequisites

Before starting this course, ensure you have:

- Basic programming knowledge in Python
- Familiarity with command-line interfaces
- Understanding of basic robotics concepts (optional but helpful)

## Environment Setup

### 1. Install Docusaurus (for viewing course locally)

```bash
# Clone the course repository
git clone <repository-url>
cd docs-site

# Install dependencies
npm install

# Start the development server
npm start
```

The course will be available at `http://localhost:3000`.

### 2. Install ROS 2 (for practical exercises)

For Ubuntu (recommended):
```bash
# Add ROS 2 repository
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Hawksbill
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep2
sudo rosdep init
rosdep update

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
```

For other platforms, see the [official ROS 2 installation guide](https://docs.ros.org/en/humble/Installation.html).

## Course Navigation

The course is organized into three progressive chapters:

1. **ROS 2 Communication Fundamentals** - Learn about nodes, topics, services, and actions
2. **Python Agents to Robot Control** - Connect Python AI agents to ROS controllers using rclpy
3. **Robot Body Representation (URDF)** - Model humanoid robots using Unified Robot Description Format

## Running Examples

Each chapter includes practical examples. To run Python examples:

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Create a workspace
mkdir -p ~/ros2_course_ws/src
cd ~/ros2_course_ws

# Build workspace
colcon build

# Source the workspace
source install/setup.bash

# Run example (replace with actual example filename)
python3 src/my_robot_examples/my_robot_examples/simple_publisher.py
```

## Key ROS 2 Commands

- `ros2 node list` - List all active nodes
- `ros2 topic list` - List all active topics
- `ros2 service list` - List all available services
- `ros2 action list` - List all available actions
- `ros2 topic echo <topic_name>` - Monitor messages on a topic
- `ros2 run <package_name> <executable>` - Run a ROS 2 node

## Troubleshooting

### Common Issues

1. **ROS 2 commands not found**: Make sure you've sourced the ROS 2 environment with `source /opt/ros/humble/setup.bash`

2. **Python packages not found**: Ensure you've installed the necessary Python packages with `pip3 install` if needed

3. **Permission errors**: Make sure you have proper permissions for ROS 2 directories

### Getting Help

- Check the [ROS 2 documentation](https://docs.ros.org/en/humble/)
- Visit the [ROS 2 community forum](https://discourse.ros.org/)
- Use `ros2 --help` for command-specific help

## Next Steps

After completing this quickstart:

1. Begin with Chapter 1: ROS 2 Communication Fundamentals
2. Follow along with the examples in your ROS 2 environment
3. Practice with the exercises at the end of each chapter
4. Explore additional ROS 2 packages and tutorials
# Digital Twin Module Quickstart Guide

## Overview
This quickstart guide provides a fast path to getting your first digital twin simulation running with Gazebo and Unity.

## Prerequisites
- ROS 2 Humble Hawksbill (or latest LTS)
- Gazebo Garden or Fortress
- Unity 2022.3 LTS
- Ubuntu 22.04 (recommended) or Windows 10/11 with WSL2

## Installation Steps

### 1. Install Gazebo and ROS 2 Packages
```bash
# Install Gazebo Garden
sudo apt update
sudo apt install ros-humble-gazebo-*

# Install ROS 2 Gazebo plugins
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control

# Install robot simulation packages
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
```

### 2. Verify Gazebo Installation
```bash
# Launch Gazebo
gz sim

# Or launch with ROS 2 bridge
ros2 launch gazebo_ros gazebo.launch.py
```

### 3. Set Up Unity Environment
1. Download and install Unity Hub from unity3d.com
2. Install Unity 2022.3 LTS through Unity Hub
3. Create a new 3D project named "DigitalTwin"
4. Import the ROS-TCP-Connector package:
   - Go to Window → Package Manager
   - Click the + icon → Add package from git URL
   - Enter: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git`

### 4. Create Basic Robot Model
```bash
# Create a simple URDF robot
mkdir -p ~/digital_twin_ws/src/my_robot_description
cd ~/digital_twin_ws/src/my_robot_description

# Create basic URDF file
cat << EOF > my_robot.urdf
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.004" ixy="0.0" ixz="0.0" iyy="0.004" iyz="0.0" izz="0.004"/>
    </inertial>
  </link>

  <joint name="head_joint" type="fixed">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.3"/>
  </joint>
</robot>
EOF
```

### 5. Launch Your First Simulation
```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Build your workspace
cd ~/digital_twin_ws
colcon build

# Source the workspace
source install/setup.bash

# Launch the robot in Gazebo
ros2 launch gazebo_ros spawn_entity.py entity:=simple_humanoid -file ./my_robot.urdf
```

### 6. Connect Unity Visualization
1. In Unity, create a new script called `RobotController.cs`
2. Add this basic connection code:

```csharp
using UnityEngine;
using ROS2;

public class RobotController : MonoBehaviour
{
    ROS2UnityComponent ros2U;
    string robotNamespace = "simple_humanoid";

    void Start()
    {
        ros2U = GetComponent<ROS2UnityComponent>();
        ros2U.ROS2ServerURL = "localhost";
        ros2U.ROS2ServerPort = "8888";
        ros2U.Init();

        // Subscribe to robot state
        ros2U.Subscribe<JointStateMsg>(robotNamespace + "/joint_states",
            (JointStateMsg msg) => {
                // Update Unity visualization based on joint states
                UpdateRobotVisualization(msg);
            });
    }

    void UpdateRobotVisualization(JointStateMsg msg)
    {
        // Implement visualization logic here
        // Update Unity transforms based on joint positions
    }
}
```

### 7. Run the Complete Simulation
1. Start ROS 2 daemon: `ros2 daemon start`
2. Launch Gazebo with your robot
3. Start Unity project with ROS connection
4. Control the robot using ROS 2 commands and observe in both Gazebo and Unity

## Troubleshooting
- If Gazebo doesn't launch: Check graphics drivers and install mesa-utils
- If ROS 2 connection fails: Verify network settings and firewall rules
- If Unity crashes: Ensure sufficient RAM and GPU memory

## Next Steps
- Explore Chapter 1: Gazebo Physics and ROS 2 Integration
- Move to Chapter 2: Unity for High-Fidelity Rendering
- Learn Chapter 3: Simulated Sensors
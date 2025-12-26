# Environment Setup Checklist for Digital Twin Module

## Pre-requisites Verification
- [ ] ROS 2 Humble Hawksbill installed and sourced
- [ ] Gazebo Garden or Fortress installed and tested
- [ ] Unity 2022.3 LTS installed via Unity Hub
- [ ] Adequate hardware specifications met:
  - [ ] CPU: Multi-core processor (4+ cores recommended)
  - [ ] RAM: 16GB minimum, 32GB recommended
  - [ ] GPU: Dedicated graphics card with 4GB+ VRAM
  - [ ] Storage: 20GB free space for simulation assets
- [ ] Network connectivity verified for ROS 2 communication

## Gazebo Installation Verification
- [ ] Gazebo GUI launches without errors (`gz sim`)
- [ ] ROS 2 Gazebo packages installed:
  - [ ] `ros-humble-gazebo-ros-pkgs`
  - [ ] `ros-humble-gazebo-ros2-control`
  - [ ] `ros-humble-ros2-control`
  - [ ] `ros-humble-ros2-controllers`
- [ ] Gazebo plugins accessible:
  - [ ] `libgazebo_ros_init.so`
  - [ ] `libgazebo_ros_factory.so`
  - [ ] `libgazebo_ros_force.so`
- [ ] Basic robot model spawns in Gazebo

## Unity Setup Verification
- [ ] Unity Hub installed and accessible
- [ ] Unity 2022.3 LTS project created
- [ ] ROS-TCP-Connector package imported
- [ ] Unity project builds without compilation errors
- [ ] Network settings configured for ROS communication

## ROS 2 Communication Verification
- [ ] ROS 2 daemon running (`ros2 daemon start`)
- [ ] Network configuration allows localhost communication
- [ ] Basic ROS 2 commands work (`ros2 topic list`, `ros2 node list`)
- [ ] Gazebo ROS bridge connects successfully
- [ ] Unity ROS connector connects to ROS master

## Basic Integration Test
- [ ] Simple URDF robot model created
- [ ] Robot spawns in Gazebo simulation
- [ ] Joint states published to ROS topics
- [ ] Unity receives robot state via ROS bridge
- [ ] Robot visualization updates in Unity

## Performance Baseline
- [ ] Gazebo achieves real-time factor ≥ 0.8x with basic scene
- [ ] Unity maintains >30 FPS with basic visualization
- [ ] Network latency <10ms between components
- [ ] System resource usage within acceptable limits:
  - [ ] CPU usage <80% average
  - [ ] GPU usage <85% average
  - [ ] Memory usage <80% of available RAM

## Troubleshooting Preparedness
- [ ] Graphics driver updated to latest version
- [ ] Firewall rules configured for ROS communication
- [ ] Backup configuration files saved
- [ ] Known issue documentation reviewed
- [ ] Support channels identified for each component
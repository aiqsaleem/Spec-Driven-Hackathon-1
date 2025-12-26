# API Contracts for Digital Twin Module

## Overview
This document defines the API contracts between different components in the Digital Twin module, including Gazebo, ROS 2, and Unity.

## 1. Gazebo-ROS 2 Bridge API

### 1.1 Publisher Topics

#### Joint States
- **Topic:** `/joint_states`
- **Type:** `sensor_msgs/JointState`
- **Frequency:** 50 Hz (configurable)
- **Description:** Publishes current joint positions, velocities, and efforts

```yaml
header:
  stamp:
    sec: int32
    nanosec: uint32
  frame_id: string
name: string[]  # Joint names
position: float64[]  # Joint positions [rad or m]
velocity: float64[]  # Joint velocities [rad/s or m/s]
effort: float64[]  # Joint efforts [Nm or N]
```

#### TF Transforms
- **Topic:** `/tf` and `/tf_static`
- **Type:** `tf2_msgs/TFMessage`
- **Frequency:** 50 Hz
- **Description:** Publishes coordinate transforms between frames

#### Sensor Data Topics
- **LiDAR:** `/scan` - `sensor_msgs/LaserScan`
- **IMU:** `/imu/data` - `sensor_msgs/Imu`
- **Camera:** `/camera/image_raw` - `sensor_msgs/Image`
- **Depth:** `/camera/depth/image_raw` - `sensor_msgs/Image`

### 1.2 Subscriber Topics

#### Joint Commands
- **Topic:** `/position_commands`, `/velocity_commands`, `/effort_commands`
- **Type:** `control_msgs/msg/JointTrajectoryController`
- **Frequency:** 100 Hz (real-time critical)
- **Description:** Receives joint position, velocity, or effort commands

#### Robot State Commands
- **Topic:** `/set_joint_positions` (service)
- **Type:** Custom service
- **Description:** Sets joint positions directly (for simulation reset)

### 1.3 Services

#### Spawn Entity
- **Service:** `/spawn_entity`
- **Type:** `gazebo_msgs/srv/SpawnEntity`
- **Description:** Spawns a new entity in the simulation

#### Delete Entity
- **Service:** `/delete_entity`
- **Type:** `gazebo_msgs/srv/DeleteEntity`
- **Description:** Removes an entity from the simulation

#### Get Physics Properties
- **Service:** `/get_physics_properties`
- **Type:** `gazebo_msgs/srv/GetPhysicsProperties`
- **Description:** Retrieves current physics engine configuration

## 2. Unity-ROS Bridge API

### 2.1 TCP Communication Protocol

#### Message Format
```
{
  "op": string,           # Operation type
  "id": string,           # Unique message ID
  "topic": string,        # Topic name (for pub/sub)
  "service": string,      # Service name (for services)
  "type": string,         # Message type
  "args": object          # Operation-specific arguments
}
```

### 2.2 Publisher Topics (Unity → ROS)

#### Robot State Updates
- **Topic:** `/unity_robot_state`
- **Type:** Custom message based on `sensor_msgs/JointState`
- **Frequency:** 30 Hz (visualization rate)
- **Description:** Sends Unity-rendered robot state for visualization

#### User Interaction Events
- **Topic:** `/unity_user_input`
- **Type:** `std_msgs/String`
- **Frequency:** Event-driven
- **Description:** Publishes user interaction events from Unity UI

### 2.3 Subscriber Topics (Unity ← ROS)

#### Robot State Subscription
- **Topic:** `/joint_states`
- **Type:** `sensor_msgs/JointState`
- **Frequency:** 50 Hz
- **Description:** Receives robot joint states for Unity visualization

#### Sensor Data Subscription
- **Topic:** Various sensor topics
- **Type:** Appropriate sensor message types
- **Frequency:** Sensor-specific
- **Description:** Receives sensor data for visualization

### 2.4 Services (Unity ↔ ROS)

#### Unity Scene Control
- **Service:** `/unity/load_scene`
- **Type:** Custom service
- **Description:** Loads a specific Unity scene

#### Visualization Configuration
- **Service:** `/unity/configure_visualization`
- **Type:** Custom service
- **Description:** Configures visualization parameters

## 3. Simulation Configuration API

### 3.1 Parameters Server

#### Physics Parameters
- **`/physics/real_time_factor`**: float64, default 1.0
- **`/physics/max_step_size`**: float64, default 0.001
- **`/physics/gravity_x`**: float64, default 0.0
- **`/physics/gravity_y`**: float64, default 0.0
- **`/physics/gravity_z`**: float64, default -9.8

#### Robot Parameters
- **`/robot/description`**: string, robot URDF
- **`/robot/controller_config`**: string, controller configuration
- **`/robot/sensor_config`**: string, sensor configuration

#### Unity Parameters
- **`/unity/visualization_rate`**: int32, default 30
- **`/unity/rendering_quality`**: string, default "medium"
- **`/unity/network_port`**: int32, default 8888

## 4. Error Handling Contracts

### 4.1 Error Response Format
```yaml
error:
  code: int32          # Error code
  message: string      # Human-readable error message
  details: object      # Additional error details (optional)
  timestamp: float64   # Time of error occurrence
```

### 4.2 Common Error Codes
- **0**: SUCCESS - Operation completed successfully
- **1**: INVALID_ARGUMENT - Invalid input parameters
- **2**: RESOURCE_UNAVAILABLE - Required resource not available
- **3**: TIMEOUT - Operation timed out
- **4**: CONNECTION_FAILED - Network connection failed
- **5**: SIMULATION_ERROR - Simulation-specific error
- **6**: VALIDATION_FAILED - Configuration validation failed

## 5. Performance Contracts

### 5.1 Timing Requirements
- **Joint state update rate:** Minimum 50 Hz
- **Control command response:** Maximum 10 ms latency
- **Sensor data publication:** As fast as sensor rate allows
- **Unity-ROS communication:** Maximum 16 ms latency (60 FPS equivalent)

### 5.2 Quality of Service (QoS) Settings
- **Joint states:** Reliable delivery, 10 message history
- **Sensor data:** Best effort, 5 message history (for visualization)
- **Control commands:** Reliable delivery, keep last message
- **Services:** Reliable delivery, request-response pattern

## 6. Security Contracts

### 6.1 Network Security
- All ROS 2 communication on local network only
- Unity-ROS bridge communication authenticated via tokens
- Configuration parameters validated before application

### 6.2 Data Validation
- All incoming messages validated against schema
- Joint limits enforced in simulation
- Invalid sensor readings filtered or flagged
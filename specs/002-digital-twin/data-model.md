# Digital Twin Module Data Model

## Overview
This document defines the data structures, schemas, and information flows for the Digital Twin module covering Gazebo and Unity integration.

## 1. Core Data Structures

### 1.1 Robot State Representation
```
RobotState {
  timestamp: float64          # Unix timestamp in seconds
  joints: map[string]JointState  # Joint name to state mapping
  links: map[string]LinkState    # Link name to state mapping
  sensors: map[string]SensorData # Sensor name to data mapping
  pose: Pose                   # Global pose in world coordinates
  twist: Twist                 # Velocity in world coordinates
}
```

### 1.2 Joint State Structure
```
JointState {
  position: float64           # Joint position (rad/m)
  velocity: float64           # Joint velocity (rad/s/m/s)
  effort: float64             # Joint effort (Nm/N)
  commanded_position: float64 # Desired position
  commanded_velocity: float64 # Desired velocity
  commanded_effort: float64   # Desired effort
}
```

### 1.3 Link State Structure
```
LinkState {
  pose: Pose                 # Position and orientation
  twist: Twist               # Linear and angular velocity
  wrench: Wrench             # Force and torque
  mass: float64              # Mass of the link
  inertia: Matrix3x3         # Inertia matrix
}
```

### 1.4 Pose Structure
```
Pose {
  position: Vector3          # x, y, z coordinates
  orientation: Quaternion    # w, x, y, z quaternion
}
```

## 2. Sensor Data Models

### 2.1 LiDAR Data Structure
```
LaserScan {
  header: Header             # Timestamp and frame ID
  angle_min: float32         # Start angle of scan [rad]
  angle_max: float32         # End angle of scan [rad]
  angle_increment: float32   # Angular distance between measurements [rad]
  time_increment: float32    # Time between measurements [seconds]
  scan_time: float32         # Time between scans [seconds]
  range_min: float32         # Minimum range value [m]
  range_max: float32         # Maximum range value [m]
  ranges: float32[]          # Range data [m]
  intensities: float32[]     # Intensity data (optional)
  # Noise parameters
  range_noise_std: float32   # Standard deviation of range noise
  angle_noise_std: float32   # Standard deviation of angle noise
}
```

### 2.2 Depth Camera Data Structure
```
DepthImage {
  header: Header             # Timestamp and frame ID
  height: uint32             # Image height
  width: uint32              # Image width
  encoding: string           # Pixel encoding (e.g., "32FC1")
  is_bigendian: bool         # Endianness flag
  step: uint32               # Full row length in bytes
  data: uint8[]              # Actual image data
  # Camera parameters
  fx: float32                # Focal length x (pixels)
  fy: float32                # Focal length y (pixels)
  cx: float32                # Principal point x (pixels)
  cy: float32                # Principal point y (pixels)
  k1, k2, k3: float32       # Radial distortion coefficients
  p1, p2: float32           # Tangential distortion coefficients
  # Depth-specific parameters
  depth_scale: float32       # Scale factor to convert pixel values to meters
  min_depth: float32         # Minimum depth value [m]
  max_depth: float32         # Maximum depth value [m]
}
```

### 2.3 IMU Data Structure
```
ImuData {
  header: Header             # Timestamp and frame ID
  orientation: Quaternion    # Orientation estimate
  orientation_covariance: float64[9]  # Covariance matrix for orientation
  angular_velocity: Vector3  # Angular velocity (rad/s)
  angular_velocity_covariance: float64[9]  # Covariance matrix for angular velocity
  linear_acceleration: Vector3  # Linear acceleration (m/s^2)
  linear_acceleration_covariance: float64[9]  # Covariance matrix for acceleration
  # Calibration parameters
  accel_bias: Vector3        # Accelerometer bias
  gyro_bias: Vector3         # Gyroscope bias
  accel_scale: Vector3       # Accelerometer scale factors
  gyro_scale: Vector3        # Gyroscope scale factors
  temperature: float32       # Sensor temperature (if available)
}
```

## 3. Simulation Configuration Models

### 3.1 Physics Configuration
```
PhysicsConfig {
  engine: string             # Physics engine (ode, bullet, dart)
  real_time_factor: float64  # Target simulation speed ratio
  max_step_size: float64     # Maximum physics step size (sec)
  gravity: Vector3           # Gravity vector (m/s^2)
  solver_type: string        # Solver type (world, quick)
  solver_iterations: int32   # Number of solver iterations
  sor: float32               # Successive over-relaxation parameter
  contact_surface_layer: float32  # Contact surface layer thickness
  contact_max_correcting_vel: float32  # Maximum contact correction velocity
}
```

### 3.2 Collision Properties
```
CollisionProperties {
  link_name: string          # Name of the link containing collision
  geometry_type: string      # Type of geometry (box, sphere, cylinder, mesh)
  geometry_params: map[string]float64  # Geometry-specific parameters
  material_coefficients: MaterialCoefficients
  bitmask: uint16            # Collision bitmask for filtering
}

MaterialCoefficients {
  mu: float32                # Friction coefficient (Coulomb)
  mu2: float32               # Secondary friction coefficient
  fdir1: Vector3             # Primary friction direction
  slip1: float32             # Primary slip coefficient
  slip2: float32             # Secondary slip coefficient
  kp: float32                # Contact stiffness
  kd: float32                # Contact damping
  max_vel: float32           # Maximum contact correcting velocity
  min_depth: float32         # Minimum contact depth
}
```

## 4. Unity Integration Data Models

### 4.1 Scene Configuration
```
SceneConfig {
  name: string               # Scene identifier
  environment: EnvironmentData  # Environment settings
  lighting: LightingConfig   # Lighting configuration
  rendering: RenderingConfig # Rendering settings
  robot_models: RobotModelConfig[]  # Configured robot models
  sensor_visualizations: SensorVizConfig[]  # Sensor visualization configs
}
```

### 4.2 Environment Data
```
EnvironmentData {
  skybox: SkyboxConfig       # Skybox settings
  terrain: TerrainConfig     # Terrain properties
  obstacles: ObstacleConfig[]  # Static and dynamic obstacles
  lighting: LightConfig[]    # Light sources in the scene
  physics_materials: map[string]PhysicsMaterial  # Physics materials
}

SkyboxConfig {
  type: string               # Type of skybox (gradient, hdr, procedural)
  colors: Color[3]           # Colors for gradient skybox
  cubemap_path: string       # Path to HDR cubemap
  exposure: float32          # Exposure adjustment
  rotation: float32          # Rotation around Y axis
}

TerrainConfig {
  heightmap_path: string     # Path to heightmap texture
  size: Vector3              # Terrain dimensions
  detail_resolution: int32   # Resolution of terrain details
  alphamap_resolution: int32 # Resolution of alphamap
  splat_prototypes: SplatPrototype[]  # Textures for terrain
}
```

### 4.3 ROS Bridge Message Formats
```
RosBridgeMessage {
  op: string                 # Operation type (publish, subscribe, call_service, etc.)
  id: string                 # Unique message identifier
  topic: string              # Topic name (for pub/sub)
  service: string            # Service name (for services)
  type: string               # Message/service type
  args: map[string]interface{}  # Operation-specific arguments
}

UnityRobotState {
  robot_id: string           # Unique robot identifier
  timestamp: float64         # Message timestamp
  joint_positions: map[string]float64  # Joint position mapping
  joint_velocities: map[string]float64 # Joint velocity mapping
  joint_efforts: map[string]float64    # Joint effort mapping
  link_poses: map[string]Pose          # Link pose mapping
  sensor_data: map[string]SensorData   # Sensor data mapping
  simulation_state: SimulationState    # Simulation-specific state
}
```

## 5. Data Flow Patterns

### 5.1 Gazebo to ROS 2 Bridge
```
Gazebo World -> Physics Update -> Sensor Generation -> ROS Message -> Publishers
```

### 5.2 ROS 2 to Unity Bridge
```
ROS Topics -> TCP Connection -> Unity Messages -> Component Updates -> Rendering
```

### 5.3 Unity to ROS 2 Bridge
```
Unity Input -> TCP Connection -> ROS Messages -> Publishers -> Robot Controllers
```

## 6. Serialization Formats

### 6.1 Configuration Files
- **JSON**: Human-readable configuration files for simulation parameters
- **YAML**: Structured configuration with inheritance capabilities
- **URDF/SDF**: Robot and world description formats

### 6.2 Data Logging
- **ROS 2 Bags**: Standardized format for recording sensor and state data
- **CSV**: Simple tabular format for numerical analysis
- **Custom Binary**: Optimized format for high-frequency data

## 7. Performance Metrics Schema

### 7.1 Simulation Performance Data
```
PerformanceMetrics {
  timestamp: float64         # Measurement timestamp
  real_time_factor: float64  # Current real-time factor
  cpu_usage: float32         # CPU utilization percentage
  gpu_usage: float32         # GPU utilization percentage
  memory_usage: uint64       # Memory usage in bytes
  physics_update_rate: float32  # Physics update rate (Hz)
  rendering_rate: float32    # Rendering frame rate (FPS)
  network_latency: float32   # Network round-trip time (ms)
  synchronization_error: float32  # Sync error between engines (ms)
}
```

### 7.2 Accuracy Metrics
```
AccuracyMetrics {
  timestamp: float64         # Measurement timestamp
  position_error: float32    # Position error compared to ground truth
  orientation_error: float32 # Orientation error (angular)
  velocity_error: float32    # Velocity error
  sensor_accuracy: map[string]float32  # Per-sensor accuracy metrics
  physics_fidelity: float32  # Overall physics simulation fidelity score
}
```
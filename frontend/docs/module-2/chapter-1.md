---
sidebar_position: 2
title: "Chapter 1: Gazebo Physics, Gravity, Collisions, and ROS 2 Integration"
---

# Chapter 1: Gazebo Physics, Gravity, Collisions, and ROS 2 Integration

## Introduction

Gazebo is a 3D dynamic simulator widely used in robotics research and development. It provides accurate physics simulation, high-quality graphics rendering, and convenient programmatic interfaces. In this chapter, we'll explore the core physics simulation capabilities of Gazebo and how they integrate with ROS 2 for comprehensive robotics applications.

### Key Features of Gazebo
- Multi-body physics simulation with multiple engine options (ODE, Bullet, DART)
- Realistic rendering with support for various sensors (cameras, LiDAR, IMU)
- Flexible world description using SDF (Simulation Description Format)
- Built-in plugins for robot control and sensor simulation
- Seamless integration with ROS 2 through the Gazebo ROS package

## Physics Engine Fundamentals

### Physics Engine Selection

Gazebo supports three different physics engines, each with specific advantages:

#### ODE (Open Dynamics Engine)
- **Strengths:** Stable, well-tested, good for ground vehicles
- **Use Cases:** Ground robots, manipulators with simple interactions
- **Configuration:** Default engine in most Gazebo installations

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <gravity>0 0 -9.8</gravity>
</physics>
```

#### Bullet Physics
- **Strengths:** Modern, good for complex contact scenarios
- **Use Cases:** Robots with complex interactions, humanoid robots
- **Configuration:** Requires Bullet installation

```xml
<physics type="bullet">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <gravity>0 0 -9.8</gravity>
  <bullet>
    <solver>
      <type>sequential_impulse</type>
      <iterations>50</iterations>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_surface_layer>0.001</contact_surface_layer>
      <contact_max_correcting_vel>100</contact_max_correcting_vel>
    </constraints>
  </bullet>
</physics>
```

#### DART (Dynamic Animation and Robotics Toolkit)
- **Strengths:** Advanced contact handling, articulated body support
- **Use Cases:** Complex humanoid robots, biomechanical simulations
- **Configuration:** Requires DART installation

### Physics Parameters

#### Time Stepping
The physics simulation advances in discrete time steps, which affects both accuracy and performance:

```xml
<max_step_size>0.001</max_step_size>  <!-- Physics step size in seconds -->
<real_time_update_rate>1000</real_time_update_rate>  <!-- Updates per second -->
```

- **Smaller step size:** More accurate but computationally expensive
- **Larger step size:** Faster but may cause instability

#### Real-Time Factor
Controls the simulation speed relative to real time:

```xml
<real_time_factor>1.0</real_time_factor>  <!-- 1x real-time speed -->
<real_time_factor>0.5</real_time_factor>  <!-- 0.5x real-time speed -->
<real_time_factor>2.0</real_time_factor>  <!-- 2x real-time speed -->
```

### Solver Configuration

The physics solver determines how forces and constraints are resolved:

```xml
<solver>
  <type>quick</type>  <!-- or "world" -->
  <iters>1000</iters>  <!-- Number of solver iterations -->
  <sor>1.3</sor>      <!-- Successive over-relaxation parameter -->
</solver>
```

- **Iterations:** More iterations = more accurate but slower
- **SOR:** Damping factor to prevent oscillation (typically 1.0-1.9)

## Gravity and Environmental Forces

### Global Gravity Configuration

Gravity is a fundamental force in physics simulation that affects all objects:

```xml
<gravity>0 0 -9.8</gravity>  <!-- Standard Earth gravity: 9.8 m/s² downward -->
```

For different celestial bodies:
- **Moon:** `<gravity>0 0 -1.62</gravity>` (1.62 m/s²)
- **Mars:** `<gravity>0 0 -3.71</gravity>` (3.71 m/s²)
- **Zero-g:** `<gravity>0 0 0</gravity>`

### Custom Force Application

You can apply custom forces to simulate environmental effects:

```xml
<model name="drone">
  <!-- Wind forces can be simulated with external force plugins -->
  <plugin name="wind_plugin" filename="libgazebo_wind_plugin.so">
    <force>0.1 0.0 0.0</force>  <!-- Constant wind force -->
    <turbulence>0.05</turbulence>  <!-- Turbulence factor -->
  </plugin>
</model>
```

### Buoyancy Simulation

For underwater robotics, buoyancy can be simulated using plugins:

```xml
<model name="underwater_robot">
  <link name="base_link">
    <inertial>
      <mass>10.0</mass>
      <inertia>
        <ixx>0.4</ixx>
        <ixy>0.0</ixy>
        <ixz>0.0</ixz>
        <iyy>0.4</iyy>
        <iyz>0.0</iyz>
        <izz>0.4</izz>
      </inertia>
    </inertial>
    <collision name="collision">
      <geometry>
        <box>
          <size>1.0 1.0 1.0</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>1.0 1.0 1.0</size>
        </box>
      </geometry>
    </visual>
  </link>

  <!-- Buoyancy plugin -->
  <plugin name="buoyancy" filename="libBuoyancyPlugin.so">
    <fluid_density>1000.0</fluid_density>  <!-- Water density -->
    <volume>1.0</volume>  <!-- Displaced volume -->
    <center_of_buoyancy>0 0 0</center_of_buoyancy>
  </plugin>
</model>
```

## Collision Detection and Response

### Collision Geometry Types

Gazebo supports various collision geometries:

#### Primitive Shapes
```xml
<collision name="box_collision">
  <geometry>
    <box>
      <size>0.5 0.5 0.2</size>
    </box>
  </geometry>
</collision>

<collision name="sphere_collision">
  <geometry>
    <sphere>
      <radius>0.1</radius>
    </sphere>
  </geometry>
</collision>

<collision name="cylinder_collision">
  <geometry>
    <cylinder>
      <radius>0.1</radius>
      <length>0.5</length>
    </cylinder>
  </geometry>
</collision>
```

#### Mesh Collisions
```xml
<collision name="mesh_collision">
  <geometry>
    <mesh>
      <uri>model://my_robot/meshes/link.stl</uri>
      <scale>1.0 1.0 1.0</scale>
    </mesh>
  </geometry>
</collision>
```

### Material Properties

#### Friction Coefficients
```xml
<collision name="wheel_collision">
  <surface>
    <friction>
      <ode>
        <mu>0.7</mu>    <!-- Primary friction coefficient -->
        <mu2>0.5</mu2>  <!-- Secondary friction coefficient -->
        <fdir1>1 0 0</fdir1>  <!-- Friction direction -->
      </ode>
      <bullet>
        <friction>0.7</friction>
        <friction2>0.5</friction2>
        <fdir1>1 0 0</fdir1>
      </bullet>
    </friction>
  </surface>
</collision>
```

#### Contact Properties
```xml
<collision name="bouncy_ball">
  <surface>
    <bounce>
      <restitution_coefficient>0.8</restitution_coefficient>  <!-- Bounciness -->
      <threshold>10.0</threshold>  <!-- Velocity threshold for bounce -->
    </bounce>
    <contact>
      <ode>
        <kp>1000000000000.0</kp>  <!-- Contact stiffness -->
        <kd>1.0</kd>              <!-- Contact damping -->
        <max_vel>100.0</max_vel>   <!-- Maximum contact correction velocity -->
        <min_depth>0.001</min_depth>  <!-- Minimum contact depth -->
      </ode>
    </contact>
  </surface>
</collision>
```

### Collision Filtering

Use collision bitmasks to control which objects collide:

```xml
<collision name="collision">
  <surface>
    <contact>
      <collide_without_contact>0</collide_without_contact>
      <collide_without_contact_bitmask>1</collide_without_contact_bitmask>
    </contact>
  </surface>
</collision>
```

## ROS 2 Integration

### Gazebo ROS Packages

The Gazebo ROS packages provide seamless integration between Gazebo and ROS 2:

#### Core Packages
- `gazebo_ros_pkgs`: Core ROS 2 plugins and launch files
- `gazebo_ros2_control`: ROS 2 control integration
- `ros_gz`: New bridge between ROS 2 and Ignition Gazebo

### Launching Gazebo with ROS 2

#### Basic Launch
```xml
<!-- launch/gazebo.launch.py -->
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                ])
            ]),
            launch_arguments={
                'world': PathJoinSubstitution([
                    FindPackageShare('my_robot_gazebo'),
                    'worlds',
                    'my_world.sdf'
                ])
            }.items()
        )
    ])
```

### ROS 2 Plugins for Gazebo

#### Joint State Publisher
```xml
<plugin filename="libgazebo_ros_joint_state_publisher.so" name="joint_state_publisher">
  <ros>
    <namespace>/my_robot</namespace>
    <remapping>~/out:=joint_states</remapping>
  </ros>
  <update_rate>30</update_rate>
  <joint_name>joint1</joint_name>
  <joint_name>joint2</joint_name>
</plugin>
```

#### Joint Position Controller
```xml
<plugin filename="libgazebo_ros_joint_pose_trajectory.so" name="joint_trajectory_controller">
  <ros>
    <namespace>/my_robot</namespace>
  </ros>
  <update_rate>1000</update_rate>
  <command_topic>joint_trajectory_controller/joint_trajectory</command_topic>
  <joint_name>joint1</joint_name>
</plugin>
```

### Sensor Integration

#### IMU Sensor
```xml
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <topic>imu/data</topic>
  <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
    <ros>
      <namespace>/my_robot</namespace>
      <remapping>~/out:=imu/data</remapping>
    </ros>
    <initial_orientation_as_reference>false</initial_orientation_as_reference>
    <body_name>imu_link</body_name>
  </plugin>
</sensor>
```

#### Camera Sensor
```xml
<sensor name="camera_sensor" type="camera">
  <camera name="head">
    <horizontal_fov>1.3962634</horizontal_fov>
    <image>
      <width>800</width>
      <height>600</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
  <topic>camera/image_raw</topic>
  <plugin filename="libgazebo_ros_camera.so" name="camera_plugin">
    <ros>
      <namespace>/my_robot</namespace>
      <remapping>~/image_raw:=camera/image_raw</remapping>
      <remapping>~/camera_info:=camera/camera_info</remapping>
    </ros>
    <camera_name>camera</camera_name>
    <frame_name>camera_optical_frame</frame_name>
  </plugin>
</sensor>
```

### Robot Control with ROS 2

#### Creating a Controller Configuration
```yaml
# config/my_robot_controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 1000  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    velocity_controller:
      type: velocity_controllers/JointGroupVelocityController

    position_controller:
      type: position_controllers/JointGroupPositionController

velocity_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3

position_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
```

#### Launching Controllers
```python
# launch/robot.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('my_robot_description'),
                'config',
                'my_robot.yaml'
            ])
        ]
    )

    # Controller Manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('my_robot_description'),
                'config',
                'my_robot_controllers.yaml'
            ])
        ],
        output='both'
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager
    ])
```

## Practical Exercises

### Exercise 1: Basic Robot in Gazebo
1. Create a simple differential drive robot URDF
2. Spawn the robot in an empty Gazebo world
3. Verify joint states are published to ROS 2
4. Control the robot using ROS 2 topics

### Exercise 2: Physics Parameter Tuning
1. Create a robot with wheels on a ramp
2. Adjust friction coefficients to achieve desired behavior
3. Tune physics parameters for stable simulation
4. Measure real-time factor and optimize performance

### Exercise 3: Sensor Integration
1. Add IMU and camera sensors to your robot
2. Verify sensor data is published to ROS 2 topics
3. Visualize sensor data in RViz2
4. Validate sensor accuracy against ground truth

## Troubleshooting and Best Practices

### Common Issues

#### Simulation Instability
- **Symptoms:** Robot shakes, explodes, or behaves erratically
- **Solutions:**
  - Reduce physics step size
  - Increase solver iterations
  - Check mass and inertia properties
  - Verify joint limits and types

#### Performance Problems
- **Symptoms:** Low real-time factor, frame drops
- **Solutions:**
  - Simplify collision geometries
  - Reduce update rates for non-critical sensors
  - Use primitive shapes instead of complex meshes
  - Optimize visual geometries separately

#### ROS 2 Communication Issues
- **Symptoms:** No joint states, unresponsive controls
- **Solutions:**
  - Verify ROS domain IDs match
  - Check network configuration
  - Confirm plugin names and parameters
  - Validate topic remappings

### Best Practices

#### Physics Configuration
- Start with conservative parameters and optimize gradually
- Use realistic mass and inertia properties
- Set appropriate joint limits and safety controllers
- Validate physics behavior against real-world data

#### Model Design
- Use simplified collision geometries for performance
- Ensure proper mass distribution
- Validate kinematic chains before dynamics
- Test individual components separately

#### Integration Testing
- Test components in isolation first
- Validate sensor data accuracy
- Monitor simulation performance metrics
- Document configuration parameters for reproducibility
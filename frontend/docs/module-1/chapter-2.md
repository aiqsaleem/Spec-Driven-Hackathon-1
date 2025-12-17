---
sidebar_position: 3
---

# Chapter 2: Python Agents to Robot Control

In this chapter, you will learn how to connect Python-based AI agents to ROS controllers using rclpy. This is crucial for bridging the gap between AI algorithms and physical robot control systems.

## Learning Objectives

By the end of this chapter, you will be able to:

- Use rclpy to write ROS 2 nodes in Python
- Bridge Python-based AI agents to ROS controllers
- Publish commands and subscribe to robot state information effectively
- Implement feedback loops between AI agents and robotic systems

## 1. Introduction to rclpy

rclpy is the Python client library for ROS 2. It provides a Python API for creating ROS 2 nodes, publishers, subscribers, services, and actions. This library is essential for connecting Python-based AI agents to the ROS 2 ecosystem.

### 1.1 Basic Node Structure

A basic ROS 2 node in Python follows this structure:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        # Initialize publishers, subscribers, services, etc. here

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 2. Creating Publishers and Subscribers for AI Integration

When bridging AI agents to robot control, you typically need to:

- Subscribe to sensor data from the robot
- Process this data through your AI algorithm
- Publish commands back to the robot

### 2.1 Sensor Data Subscriber

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class AIBridgeNode(Node):
    def __init__(self):
        super().__init__('ai_bridge_node')

        # Subscribe to laser scan data
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publisher for robot commands
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer for AI processing loop
        self.timer = self.create_timer(0.1, self.ai_processing_loop)

        # Store latest sensor data
        self.latest_scan = None

    def laser_callback(self, msg):
        # Store the latest laser scan data
        self.latest_scan = msg
        self.get_logger().info(f'Received laser scan with {len(msg.ranges)} ranges')

    def ai_processing_loop(self):
        if self.latest_scan is not None:
            # Process sensor data through AI algorithm
            command = self.process_ai_logic(self.latest_scan)

            # Publish the command to the robot
            self.publisher.publish(command)

    def process_ai_logic(self, scan_data):
        # Simple AI logic: move forward if no obstacles within 1 meter
        min_distance = min(scan_data.ranges)

        cmd = Twist()
        if min_distance > 1.0:
            cmd.linear.x = 0.5  # Move forward
            cmd.angular.z = 0.0
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Turn to avoid obstacle

        return cmd

def main(args=None):
    rclpy.init(args=args)
    ai_bridge = AIBridgeNode()

    try:
        rclpy.spin(ai_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        ai_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3. Robot State Subscription

To create effective AI agents, you need to subscribe to various robot state information:

- Sensor data (LIDAR, cameras, IMU, etc.)
- Robot pose and odometry
- Joint states
- Battery levels
- Other status information

### 3.1 Multiple Subscriptions Example

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, BatteryState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class AdvancedAIBridgeNode(Node):
    def __init__(self):
        super().__init__('advanced_ai_bridge')

        # Multiple subscriptions for different sensor data
        self.laser_subscription = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10)

        self.odom_subscription = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)

        self.battery_subscription = self.create_subscription(
            BatteryState, 'battery_status', self.battery_callback, 10)

        # Publisher for robot commands
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Store robot state
        self.robot_state = {
            'position': None,
            'orientation': None,
            'battery_level': 100.0,
            'obstacle_distances': []
        }

        # AI processing timer
        self.ai_timer = self.create_timer(0.2, self.run_ai_logic)

    def laser_callback(self, msg):
        self.robot_state['obstacle_distances'] = msg.ranges

    def odom_callback(self, msg):
        self.robot_state['position'] = msg.pose.pose.position
        self.robot_state['orientation'] = msg.pose.pose.orientation

    def battery_callback(self, msg):
        self.robot_state['battery_level'] = msg.percentage

    def run_ai_logic(self):
        # Make decisions based on the current robot state
        if self.robot_state['battery_level'] < 20.0:
            # Return to charging station if battery is low
            cmd = self.return_to_charger()
        else:
            # Navigate based on obstacle detection
            cmd = self.navigate_with_obstacles()

        self.cmd_publisher.publish(cmd)

    def return_to_charger(self):
        # Simple logic to return to charger
        cmd = Twist()
        cmd.linear.x = -0.2  # Move backward slowly
        cmd.angular.z = 0.0
        return cmd

    def navigate_with_obstacles(self):
        # Navigate avoiding obstacles
        cmd = Twist()
        if self.robot_state['obstacle_distances']:
            min_distance = min([d for d in self.robot_state['obstacle_distances'] if d > 0])
            if min_distance > 1.0:
                cmd.linear.x = 0.5  # Move forward
                cmd.angular.z = 0.0
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5  # Turn to avoid obstacle
        return cmd

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedAIBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3.2 Joint State Subscription

For robots with articulated bodies, you often need to subscribe to joint states:

```python
from sensor_msgs.msg import JointState

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')

        # Subscribe to joint states
        self.joint_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)

        # Store current joint positions
        self.joint_positions = {}

    def joint_state_callback(self, msg):
        # Update joint positions dictionary
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]

        # Log joint states
        self.get_logger().info(f'Joint positions: {self.joint_positions}')
```

### 3.3 Camera Image Subscription

For vision-based AI agents, you'll need to subscribe to camera feeds:

```python
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VisionAIAgent(Node):
    def __init__(self):
        super().__init__('vision_ai_agent')

        # Initialize OpenCV bridge
        self.bridge = CvBridge()

        # Subscribe to camera image
        self.image_subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)

        # Store latest image
        self.latest_image = None

    def image_callback(self, msg):
        try:
            # Convert ROS image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image

            # Process image with AI algorithm
            processed_image = self.process_vision_ai(cv_image)

            # Use results for robot control decisions
            self.make_control_decision(processed_image)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def process_vision_ai(self, image):
        # Example: Simple color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # ... AI processing logic here ...
        return image

    def make_control_decision(self, processed_image):
        # Convert AI results to robot commands
        pass
```

## 4. Command Publishing Strategies

When publishing commands from AI agents to robots, consider these strategies:

### 4.1 Rate Limiting

To avoid overwhelming the robot with commands, implement rate limiting:

```python
def __init__(self):
    super().__init__('ai_bridge_node')
    # ... other initialization ...

    # Rate-limited command publishing
    self.command_rate = 10  # Hz
    self.command_timer = self.create_timer(1.0/self.command_rate, self.publish_command_if_ready)
    self.last_command_time = self.get_clock().now()
    self.command_available = False

def ai_logic_callback(self):
    # Process AI logic
    new_command = self.process_ai_logic()

    # Mark that a new command is available
    self.pending_command = new_command
    self.command_available = True

def publish_command_if_ready(self):
    current_time = self.get_clock().now()
    if self.command_available and (current_time - self.last_command_time).nanoseconds > 1e9/self.command_rate:
        self.cmd_publisher.publish(self.pending_command)
        self.last_command_time = current_time
        self.command_available = False
```

### 4.2 Safety Constraints

Always implement safety constraints when sending commands to robots:

```python
def apply_safety_constraints(self, cmd):
    # Limit maximum velocities
    cmd.linear.x = max(min(cmd.linear.x, 1.0), -1.0)  # Max ±1.0 m/s
    cmd.angular.z = max(min(cmd.angular.z, 1.0), -1.0)  # Max ±1.0 rad/s

    # Check for emergency stop conditions
    if self.emergency_stop_active:
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

    return cmd
```

## 5. Python Agent Integration Patterns

### 5.1 Machine Learning Model Integration

Integrate trained ML models with ROS 2:

```python
import tensorflow as tf  # or pytorch, scikit-learn, etc.

class MLControllerNode(Node):
    def __init__(self):
        super().__init__('ml_controller')

        # Load pre-trained model
        self.model = tf.keras.models.load_model('path/to/model')

        # Subscriptions and publishers
        self.sensor_subscription = self.create_subscription(
            SensorMsgType, 'sensor_data', self.sensor_callback, 10)
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.latest_sensor_data = None

    def sensor_callback(self, msg):
        self.latest_sensor_data = msg
        if self.should_run_ml_model():
            action = self.run_ml_model()
            self.cmd_publisher.publish(action)

    def run_ml_model(self):
        # Preprocess sensor data
        processed_data = self.preprocess(self.latest_sensor_data)

        # Run inference
        prediction = self.model.predict(processed_data)

        # Convert prediction to robot command
        cmd = self.convert_prediction_to_command(prediction)

        return cmd
```

### 5.2 Behavior Trees and State Machines

Implement complex AI behaviors using structured approaches:

```python
from enum import Enum

class RobotBehavior(Enum):
    IDLE = 1
    EXPLORING = 2
    AVOIDING_OBSTACLE = 3
    RETURNING_HOME = 4

class BehaviorNode(Node):
    def __init__(self):
        super().__init__('behavior_node')
        self.current_behavior = RobotBehavior.IDLE
        # ... other initialization ...

    def update_behavior(self):
        if self.robot_state['battery_level'] < 15.0:
            self.current_behavior = RobotBehavior.RETURNING_HOME
        elif self.detect_obstacle():
            self.current_behavior = RobotBehavior.AVOIDING_OBSTACLE
        else:
            self.current_behavior = RobotBehavior.EXPLORING

    def execute_current_behavior(self):
        if self.current_behavior == RobotBehavior.IDLE:
            return self.execute_idle()
        elif self.current_behavior == RobotBehavior.EXPLORING:
            return self.execute_exploring()
        elif self.current_behavior == RobotBehavior.AVOIDING_OBSTACLE:
            return self.execute_avoiding_obstacle()
        elif self.current_behavior == RobotBehavior.RETURNING_HOME:
            return self.execute_returning_home()
```

## 6. Hands-On Exercise: AI Agent Bridge

Create a complete AI agent that bridges sensor data to robot control.

### 6.1 Exercise: Simple Navigation Agent

Create a Python script that:

1. Subscribes to laser scan data
2. Implements a simple navigation algorithm (e.g., move toward a goal while avoiding obstacles)
3. Publishes velocity commands to control the robot
4. Uses safety constraints to ensure safe operation

### 6.2 Implementation Template

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from math import pi

class NavigationAgentNode(Node):
    def __init__(self):
        super().__init__('navigation_agent')

        # TODO: Initialize subscriptions, publishers, and state variables
        # Subscribe to laser scan
        # Create publisher for velocity commands
        # Set goal position
        # Initialize robot state

    def laser_callback(self, msg):
        # TODO: Process laser scan data
        pass

    def navigation_logic(self):
        # TODO: Implement navigation algorithm
        # 1. Check if goal is reached
        # 2. If not, determine next action based on sensor data
        # 3. Apply safety constraints
        # 4. Publish command
        pass

def main(args=None):
    rclpy.init(args=args)
    node = NavigationAgentNode()

    try:
        # TODO: Add timer for navigation logic
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 7. Best Practices for AI-Robot Integration

1. **Error Handling**: Always include robust error handling for sensor failures or communication issues
2. **Safety First**: Implement safety constraints and emergency stop functionality
3. **Rate Limiting**: Don't overwhelm the robot with commands at too high a frequency
4. **State Management**: Keep track of robot state and AI decision-making context
5. **Logging**: Log important events for debugging and analysis
6. **Testing**: Test AI algorithms in simulation before deploying to real robots

## 8. Chapter Summary

In this chapter, you learned:

- How to use rclpy to create ROS 2 nodes in Python
- How to bridge Python-based AI agents to ROS controllers
- How to subscribe to sensor data and publish robot commands
- Various integration patterns for AI-robot interaction
- Best practices for safe and effective AI-robot integration

These skills are essential for connecting AI algorithms to physical robotic systems and form the core of intelligent robot behavior.

## 9. Hands-On Exercises

Complete these exercises to reinforce your understanding:

### Exercise 1: Follow the Gap Algorithm
Implement a "Follow the Gap" algorithm that finds the largest gap in laser scan data and navigates toward it while avoiding obstacles.

**Implementation Steps**:
1. Subscribe to the `/scan` topic for laser data
2. Identify the largest clear gap in the scan data
3. Steer the robot toward the center of the gap
4. Implement safety checks to avoid collisions

**Solution**: Create a file `follow_gap.py` with your implementation.

### Exercise 2: Goal Navigation
Create a node that navigates to a specified goal position while avoiding obstacles using sensor data.

**Implementation Steps**:
1. Set a goal position (x, y coordinates)
2. Subscribe to odometry data to get current robot position
3. Subscribe to laser scan data for obstacle detection
4. Implement a path planning algorithm (e.g., proportional navigation)
5. Publish velocity commands to reach the goal while avoiding obstacles

**Solution**: Create a file `goal_navigation.py` with your implementation.

### Exercise 3: Multi-Sensor Fusion
Create a node that subscribes to multiple sensor types (e.g., laser scan and camera) and makes navigation decisions based on fused sensor data.

**Implementation Steps**:
1. Subscribe to both laser scan and camera image topics
2. Process each sensor modality separately
3. Combine the information from both sensors
4. Make navigation decisions based on the fused data
5. Handle cases where one sensor might be unreliable

**Solution**: Create a file `multi_sensor_fusion.py` with your implementation.

### Exercise 4: AI Behavior Tree
Implement a simple behavior tree for robot navigation that includes:
- Sequence nodes (execute children in order until one fails)
- Selector nodes (try children until one succeeds)
- Condition nodes (check robot state)
- Action nodes (execute robot commands)

**Implementation Steps**:
1. Define basic behavior tree node types
2. Create a navigation behavior tree
3. Implement the tree execution logic
4. Test with simulated sensor data

**Solution**: Create a file `behavior_tree.py` with your implementation.

### Exercise 5: Learning Agent
Create a simple learning agent that adapts its behavior based on feedback.

**Implementation Steps**:
1. Track robot performance (e.g., time to reach goals, collisions)
2. Adjust navigation parameters based on performance
3. Implement a simple learning algorithm (e.g., Q-learning for parameter tuning)
4. Test the adaptation over multiple runs

**Solution**: Create a file `learning_agent.py` with your implementation.

## 10. Learning Validation

To validate your understanding of this chapter, try to:

1. Create a Python node that subscribes to sensor data and publishes commands based on simple AI logic
2. Implement safety constraints in your AI agent
3. Use multiple subscriptions to create a more sophisticated AI agent
4. Apply rate limiting to your command publishing

## Next Steps

In the next chapter, we'll explore robot body representation using URDF, which is essential for understanding and controlling robot kinematics in simulation and reality.
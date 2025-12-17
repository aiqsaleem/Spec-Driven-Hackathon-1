---
sidebar_position: 2
---

# Chapter 1: ROS 2 Communication Fundamentals

In this chapter, you will learn the core ROS 2 concepts including nodes, topics, services, and actions. These communication patterns form the foundation of how components in a robotic system interact with each other.

## Learning Objectives

By the end of this chapter, you will be able to:

- Explain the difference between nodes, topics, services, and actions
- Create a simple publisher-subscriber pair
- Use ROS 2 CLI tools to inspect active nodes, topics, and services
- Understand the ROS 2 communication model and DDS

## 1. Introduction to ROS 2 Architecture

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

### 1.1 Nodes

A **node** is an executable that uses ROS 2 to communicate with other nodes. Nodes are the fundamental building blocks of a ROS 2 system. They encapsulate functionality and communicate with other nodes through messages.

In Python, you create a node by inheriting from the `Node` class:

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
```

### 1.2 Topics and Message Passing

**Topics** are named buses over which nodes exchange messages. The communication is based on a publish/subscribe pattern where publishers send messages to a topic and subscribers receive messages from a topic.

Key characteristics of topics:
- Unidirectional data flow (publisher → topic → subscriber)
- Anonymous publishers and subscribers
- Multiple publishers and subscribers can exist for the same topic
- Data is distributed in a many-to-many fashion

### 1.3 Services

**Services** provide a request/response communication pattern. Unlike topics, services are synchronous - the client waits for a response from the server.

A service has two parts:
- Service definition (`.srv` file)
- Service client and service server

### 1.4 Actions

**Actions** are a more advanced communication pattern for long-running tasks. They provide feedback during execution and can be canceled.

Actions have three parts:
- Goal: Request to start a task
- Feedback: Status updates during execution
- Result: Final outcome of the task

## 2. ROS 2 Communication Model and DDS

ROS 2 uses DDS (Data Distribution Service) as its communication layer. DDS is a middleware standard that provides a publisher-subscriber communication model with quality-of-service (QoS) controls.

### 2.1 Quality of Service (QoS)

QoS settings allow you to control how messages are delivered:
- Reliability: Best effort or reliable delivery
- Durability: Volatile or transient-local
- History: Keep-all or keep-last N messages
- Deadline: Maximum time between consecutive messages

## 3. ROS 2 CLI Tools

ROS 2 provides several command-line tools for introspection and debugging:

- `ros2 node list` - List all active nodes
- `ros2 topic list` - List all active topics
- `ros2 service list` - List all available services
- `ros2 action list` - List all available actions
- `ros2 topic echo <topic_name>` - Monitor messages on a topic
- `ros2 topic info <topic_name>` - Get information about a topic
- `ros2 node info <node_name>` - Get information about a node

### Example: Inspecting ROS 2 System

```bash
# List all nodes
ros2 node list

# List all topics
ros2 topic list

# Listen to messages on a specific topic
ros2 topic echo /chatter std_msgs/msg/String

# Get detailed information about a specific topic
ros2 topic info /chatter

# List all services
ros2 service list

# Get information about a specific service
ros2 service info /add_two_ints

# List all actions
ros2 action list

# Get information about a specific action
ros2 action info /fibonacci_as
```

### Advanced CLI Tools Examples

You can also use ROS 2 CLI tools to interact with your system programmatically:

```bash
# Call a service directly from command line
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"

# Publish a message directly from command line
ros2 topic pub /chatter std_msgs/String "data: 'Hello from CLI'"

# Get information about a specific node
ros2 node info /minimal_publisher
```

## 4. Quality of Service (QoS) Settings

When creating publishers and subscribers, you can specify Quality of Service (QoS) settings to control message delivery behavior:

```python
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

# Create a QoS profile with specific settings
qos_profile = QoSProfile(
    depth=10,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.RELIABLE
)

# Use the QoS profile when creating a publisher
self.publisher = self.create_publisher(String, 'topic', qos_profile)
```

## 5. Hands-On Exercise: Simple Publisher and Subscriber

Let's create a simple publisher and subscriber to demonstrate the publish-subscribe pattern.

### 4.1 Publisher Example

Create a file `simple_publisher.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher = self.create_publisher(String, 'robot_status', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_status)
        self.status_counter = 0

    def publish_status(self):
        msg = String()
        msg.data = f'Robot status: Operational - {self.status_counter}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.status_counter += 1

def main(args=None):
    rclpy.init(args=args)
    publisher = SimplePublisher()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4.2 Subscriber Example

Create a file `simple_subscriber.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'robot_status',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    subscriber = SimpleSubscriber()

    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4.3 Running the Example

1. Open two terminal windows
2. In the first terminal, source ROS 2: `source /opt/ros/humble/setup.bash`
3. Run the publisher: `python3 simple_publisher.py`
4. In the second terminal, run the subscriber: `python3 simple_subscriber.py`

You should see the publisher sending messages and the subscriber receiving them.

## 5. Chapter Summary

In this chapter, you learned:

- The fundamental ROS 2 concepts: nodes, topics, services, and actions
- How the publish-subscribe communication pattern works
- The role of DDS in ROS 2 communication
- How to use ROS 2 CLI tools for system introspection
- How to create simple publisher and subscriber nodes

These fundamentals form the basis for more advanced ROS 2 programming and are essential for connecting AI agents to robotic systems.

## 6. Hands-On Exercises

Complete these exercises to reinforce your understanding of ROS 2 communication fundamentals:

### Exercise 1: Temperature Sensor Publisher
Create a publisher that simulates a temperature sensor. It should publish temperature readings to a topic called `temperature` using the `std_msgs/Float32` message type. The temperature should vary randomly between 18.0 and 25.0 degrees Celsius.

**Solution**: Create a file `temp_publisher.py` with your implementation.

### Exercise 2: Command Subscriber
Create a subscriber that listens to a topic called `robot_command` for string commands. When it receives a command like "move_forward", "turn_left", etc., it should log the command to the console.

**Solution**: Create a file `command_subscriber.py` with your implementation.

### Exercise 3: System Inspection
Using ROS 2 CLI tools, inspect your running system:
1. List all active nodes
2. Find a topic with a high message rate
3. Echo messages from a topic for 10 seconds
4. Get information about a specific node

**Solution**: Document the commands you used and the output you observed.

## 7. Learning Validation

To validate your understanding of this chapter, try to:

1. Explain the difference between topics, services, and actions with specific examples
2. Create a publisher-subscriber pair that communicates information about a robot's sensor readings
3. Use ROS 2 CLI tools to inspect your running nodes and topics
4. Modify the example code to publish different types of messages

## Next Steps

In the next chapter, we'll explore how to bridge Python-based AI agents to ROS controllers, building on the communication fundamentals you've learned here.
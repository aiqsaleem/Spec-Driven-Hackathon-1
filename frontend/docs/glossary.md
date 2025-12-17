---
sidebar_position: 100
---

# Glossary

This glossary defines key terms used throughout the ROS 2 fundamentals course.

## A

**Action** - A ROS 2 communication pattern for long-running tasks with feedback and status. Actions are built on top of services and provide more sophisticated request-response interactions.

**AI Agent** - An autonomous program that perceives its environment and takes actions to achieve goals. In robotics, AI agents often interface with ROS controllers to command robot behavior.

## C

**Controller** - A software component that manages robot hardware or simulated components. Controllers in ROS 2 often interface with the ros2_control framework.

**Coordinate Frame** - A reference frame in 3D space used to define positions and orientations. ROS 2 uses TF (Transform) to manage coordinate frame relationships.

## D

**DDS (Data Distribution Service)** - The middleware layer that underlies ROS 2, providing publish-subscribe communication patterns with Quality of Service (QoS) controls.

## J

**Joint** - A connection between two links in a robot model that defines how they can move relative to each other. Joint types include revolute, prismatic, continuous, and fixed.

**Joint State** - Information about the position, velocity, and effort of robot joints. Joint states are typically published on the `/joint_states` topic.

## L

**Link** - A rigid body in a robot model that has physical properties like mass, inertia, and geometry. Links are connected by joints in URDF models.

## N

**Node** - A process that performs computation in ROS. Nodes are the fundamental building blocks of ROS applications and communicate with other nodes using topics, services, and actions.

## Q

**QoS (Quality of Service)** - A set of policies that define how messages are delivered in ROS 2, including reliability, durability, and liveliness settings.

## R

**Robot Operating System (ROS)** - A flexible framework for writing robot software. ROS 2 is the second generation with improved architecture for real-world applications.

**ROS 2** - The second generation of the Robot Operating System, featuring improved security, real-time capabilities, and support for commercial products.

**rclpy** - The Python client library for ROS 2, allowing Python programs to interact with ROS 2 systems.

## S

**Service** - A ROS 2 communication pattern for request-response interactions. Services provide synchronous communication between nodes.

**Subscriber** - A node that receives messages from a topic in the publish-subscribe communication pattern.

## T

**TF (Transform)** - The system for tracking coordinate frame relationships over time in ROS. TF allows transformations between different coordinate frames.

**TF2** - The second generation of the Transform library in ROS, providing improved performance and features.

**Topic** - A named bus over which nodes exchange messages in the publish-subscribe communication pattern.

**Transform** - A mathematical operation that converts coordinates from one coordinate frame to another.

## U

**URDF (Unified Robot Description Format)** - An XML-based format for representing robot models, including links, joints, and their physical properties.

## P

**Publisher** - A node that sends messages to a topic in the publish-subscribe communication pattern.

## X

**Xacro** - XML macros for URDF, allowing parameterization and reuse of robot model components to simplify complex robot descriptions.
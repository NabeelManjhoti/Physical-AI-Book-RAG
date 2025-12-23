---
sidebar_position: 2
---

# ROS 2 Nodes, Topics, and Services

ROS 2 provides a flexible framework for distributed computing in robotics. Understanding the core communication patterns is essential for building robust robotic systems.

## Nodes

A node is a process that performs computation. Nodes are the fundamental building blocks of a ROS 2 system. Multiple nodes are typically used together to create a complete robotic application. Nodes written in different programming languages can be combined in a single system.

### Node Architecture

- **Node**: The basic execution unit of a ROS 2 program
- **Node Handles**: Provide interface to the ROS graph for a particular node
- **Execution Models**: Determine how callbacks are called

### Creating a Node

Nodes are created by extending the `Node` class and implementing the desired functionality. Each node should have a unique name within the ROS graph.

## Topics and Publishers/Subscribers

Topics enable asynchronous message passing between nodes using a publish/subscribe pattern.

### Key Concepts

- **Topic**: A named bus over which nodes exchange messages
- **Publisher**: Sends messages to a topic
- **Subscriber**: Receives messages from a topic
- **Message Types**: Strongly-typed data structures for communication

### Quality of Service (QoS)

ROS 2 provides QoS settings to control the behavior of publishers and subscribers, including reliability, durability, and history policies.

## Services

Services provide synchronous request/response communication between nodes.

### Service Architecture

- **Service Server**: Provides a service by receiving requests and sending responses
- **Service Client**: Uses a service by sending requests and receiving responses
- **Service Types**: Define the request and response message types

## Actions

Actions are used for long-running tasks that require feedback and goal management.

### Action Components

- **Goal**: Request sent to an action server
- **Feedback**: Periodic updates during action execution
- **Result**: Final outcome of the action

## Best Practices

- Use meaningful names for nodes, topics, and services
- Follow ROS 2 naming conventions
- Implement proper error handling
- Use appropriate QoS settings for your application
- Separate concerns across multiple nodes
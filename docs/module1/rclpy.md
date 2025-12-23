---
sidebar_position: 3
---

# rclpy: Python Client Library for ROS 2

rclpy is the Python client library for ROS 2, providing Python bindings to the ROS 2 client library (rcl). It enables Python developers to create ROS 2 nodes, publishers, subscribers, services, and actions.

## Core Concepts

### Node Creation

Creating a node in rclpy involves subclassing `rclpy.node.Node` and implementing the desired functionality:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        # Initialize node components here

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
```

### Publishers

Publishers allow nodes to send messages to topics:

```python
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'topic_name', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World'
        self.publisher.publish(msg)
```

### Subscribers

Subscribers receive messages from topics:

```python
class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'topic_name',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
```

### Services

Creating services in rclpy:

```python
from example_interfaces.srv import AddTwoInts

class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response
```

## Advanced Topics

### Parameters

ROS 2 nodes can use parameters for configuration:

```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        self.declare_parameter('my_parameter', 'default_value')

    def get_parameter_value(self):
        my_param = self.get_parameter('my_parameter').value
        return my_param
```

### Timers

Timers allow for periodic execution:

```python
def __init__(self):
    # ... other initialization
    self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
```

### Lifecycle Nodes

For complex systems, lifecycle nodes provide better state management:

```python
from lifecycle_py import LifecycleNode

class MyLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('lifecycle_node')
```

## Best Practices

- Always call `rclpy.init()` before creating nodes
- Use try/finally blocks for proper cleanup
- Follow ROS 2 naming conventions
- Use appropriate message types from standard ROS packages when possible
- Implement proper logging for debugging
- Consider thread safety in callbacks
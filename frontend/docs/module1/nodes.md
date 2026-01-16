---
sidebar_position: 3
---

# ROS 2 Nodes

In ROS 2, a node is a process that performs computation. Nodes are the fundamental building blocks of a ROS 2 system, and they can be distributed across multiple machines in a network.

## Understanding Nodes

Nodes are designed to be modular, meaning that each node should perform a specific task. This modularity makes ROS 2 systems easier to debug and maintain. Each node communicates with other nodes through topics, services, and actions.

## Creating a Node in Python

Let's create a more sophisticated node that includes both publishing and subscribing capabilities:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerListenerNode(Node):
    def __init__(self):
        super().__init__('talker_listener')

        # Create a publisher
        self.publisher = self.create_publisher(String, 'chatter', 10)

        # Create a subscription
        self.subscription = self.create_subscription(
            String,
            'listen',
            self.listener_callback,
            10
        )

        # Create a timer to publish messages periodically
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)

    node = TalkerListenerNode()

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

## Node Parameters

Nodes can accept parameters to customize their behavior without recompiling:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ParameterizedNode(Node):
    def __init__(self):
        super().__init__('parameterized_talker')

        # Declare parameters with default values
        self.declare_parameter('message', 'Default message')
        self.declare_parameter('frequency', 1.0)

        # Get parameter values
        self.message = self.get_parameter('message').value
        self.frequency = self.get_parameter('frequency').value

        # Create publisher
        self.publisher = self.create_publisher(String, 'parameter_chatter', 10)

        # Create timer with frequency from parameter
        timer_period = 1.0 / self.frequency
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = self.message
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)

    node = ParameterizedNode()

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

## Running Nodes

To run the node, first make sure your ROS 2 environment is sourced:

```bash
source /opt/ros/humble/setup.bash
```

Then run your Python node directly:

```bash
python3 your_node_file.py
```

Or create a launch file for managing multiple nodes:

```python
# launch/example_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package',
            executable='talker_listener',
            name='talker_listener_node',
            parameters=[
                {'message': 'Custom message'},
                {'frequency': 2.0}
            ]
        )
    ])
```

## Best Practices

- **Single Responsibility**: Each node should have a single, well-defined purpose
- **Error Handling**: Implement proper error handling and logging
- **Resource Cleanup**: Always clean up resources when the node shuts down
- **Parameter Usage**: Use parameters for configuration rather than hardcoded values
- **Naming Conventions**: Use descriptive names for nodes, topics, and services

## Exercise

Create a node that:
1. Subscribes to a topic called "input_numbers"
2. Receives integer messages
3. Publishes the square of each received number to "squared_numbers"
4. Uses a parameter to specify the maximum number to process

This exercise will help you practice creating nodes with subscriptions, publications, and parameters.

## Summary

In this section, you learned:
- How to create nodes with publishers and subscribers
- How to use parameters to configure node behavior
- Best practices for node development
- How to run nodes in the ROS 2 environment

In the next section, we'll explore topics and messages in greater detail.
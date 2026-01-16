---
sidebar_position: 4
---

# ROS 2 Topics and Messages

Topics are named buses over which nodes exchange messages. Understanding topics is crucial for creating communication between different parts of your robotic system.

## Understanding Topics

Topics in ROS 2 implement a publish-subscribe communication pattern. Publishers send messages to a topic, and subscribers receive messages from a topic. This decouples the nodes that produce data from the nodes that consume data.

## Creating Custom Messages

While ROS 2 provides many standard message types, you'll often need to create custom messages. Create a file called `NumArray.msg`:

```
int64[] data
string description
uint32 count
```

To use custom messages, you need to create a package with a `msg` directory and proper CMakeLists.txt configuration.

## Publisher Example

Here's a more complex publisher example:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist

class ComplexPublisher(Node):
    def __init__(self):
        super().__init__('complex_publisher')

        # Multiple publishers
        self.chatter_pub = self.create_publisher(String, 'chatter', 10)
        self.number_pub = self.create_publisher(Int32, 'numbers', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer for periodic publishing
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.counter = 0

    def timer_callback(self):
        # Publish string message
        string_msg = String()
        string_msg.data = f'Complex message #{self.counter}'
        self.chatter_pub.publish(string_msg)

        # Publish number message
        number_msg = Int32()
        number_msg.data = self.counter
        self.number_pub.publish(number_msg)

        # Publish velocity command
        twist_msg = Twist()
        twist_msg.linear.x = float(self.counter % 10) * 0.1  # Forward velocity
        twist_msg.angular.z = float(self.counter % 4) * 0.5  # Angular velocity
        self.cmd_vel_pub.publish(twist_msg)

        self.get_logger().info(f'Published messages #{self.counter}')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    publisher = ComplexPublisher()

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

## Subscriber Example

Here's a subscriber that handles multiple topic types:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
import json

class ComplexSubscriber(Node):
    def __init__(self):
        super().__init__('complex_subscriber')

        # Multiple subscriptions
        self.string_sub = self.create_subscription(
            String,
            'chatter',
            self.string_callback,
            10
        )

        self.number_sub = self.create_subscription(
            Int32,
            'numbers',
            self.number_callback,
            10
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Store recent messages
        self.recent_messages = []

    def string_callback(self, msg):
        self.get_logger().info(f'Received string: "{msg.data}"')
        self.add_to_recent('string', msg.data)

    def number_callback(self, msg):
        self.get_logger().info(f'Received number: {msg.data}')
        self.add_to_recent('number', msg.data)

    def cmd_vel_callback(self, msg):
        self.get_logger().info(
            f'Received velocity: linear.x={msg.linear.x}, angular.z={msg.angular.z}'
        )
        velocity_data = {
            'linear_x': msg.linear.x,
            'angular_z': msg.angular.z
        }
        self.add_to_recent('velocity', velocity_data)

    def add_to_recent(self, msg_type, content):
        self.recent_messages.append({
            'type': msg_type,
            'content': content,
            'timestamp': self.get_clock().now().seconds_nanoseconds()
        })
        # Keep only the last 10 messages
        if len(self.recent_messages) > 10:
            self.recent_messages.pop(0)

def main(args=None):
    rclpy.init(args=args)
    subscriber = ComplexSubscriber()

    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        # Print recent messages before exiting
        print("\nRecent messages:")
        for msg in subscriber.recent_messages:
            print(f"  {msg['type']}: {msg['content']}")
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Quality of Service (QoS) Settings

QoS settings allow you to control how messages are delivered:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class QoSPublisher(Node):
    def __init__(self):
        super().__init__('qos_publisher')

        # Create a QoS profile with specific settings
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,  # Ensure delivery
            durability=DurabilityPolicy.VOLATILE,    # Don't keep old messages
        )

        self.publisher = self.create_publisher(String, 'qos_chatter', qos_profile)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'QoS message #{self.counter}'
        self.publisher.publish(msg)
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = QoSPublisher()

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

## Topic Tools

ROS 2 provides command-line tools for inspecting topics:

```bash
# List all topics
ros2 topic list

# Show information about a specific topic
ros2 topic info /chatter

# Echo messages from a topic
ros2 topic echo /chatter std_msgs/msg/String

# Publish a message to a topic from command line
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello from command line'"
```

## Best Practices

- **Topic Naming**: Use descriptive, lowercase names with underscores
- **Message Types**: Choose appropriate message types for your data
- **QoS Settings**: Match QoS settings between publishers and subscribers
- **Message Size**: Be mindful of message size for network performance
- **Rate Limiting**: Avoid publishing too frequently without necessity

## Exercise

Create a system with:
1. A publisher that publishes sensor readings (temperature, humidity, pressure)
2. A subscriber that receives these readings and calculates derived values
3. A third node that subscribes to the derived values and logs them to a file

This exercise will help you practice creating a multi-node system with custom data flows.

## Summary

In this section, you learned:
- How to create publishers and subscribers for multiple topics
- How to use Quality of Service settings
- How to work with different message types
- Best practices for topic design
- Available command-line tools for debugging

In the next section, we'll explore services and actions for request-response communication patterns.
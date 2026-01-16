---
sidebar_position: 2
---

# ROS 2 Basics

This section covers the fundamental concepts of Robot Operating System 2 (ROS 2).

## What is ROS 2?

ROS 2 (Robot Operating System 2) is a set of software libraries and tools that help you build robot applications. It provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more. ROS 2 is the evolution of the original ROS framework with improved features for production use.

## Key Concepts

### Nodes
A node is a process that performs computation. ROS is designed to be a distributed system where nodes can be spread across multiple devices. In Python, you define a node by subclassing `rclpy.Node`.

### Topics and Messages
Topics are named buses over which nodes exchange messages. A node can publish messages to a topic or subscribe to a topic to receive messages. Messages are the data sent between nodes.

### Services
Services provide a request/reply communication pattern. A service client sends a request message and waits for a response message.

### Actions
Actions are like services but designed for long-running tasks. They include feedback during execution and can be canceled.

## Installing ROS 2

For this course, we recommend installing ROS 2 Humble Hawksbill LTS on Ubuntu 22.04:

```bash
# Add the ROS 2 apt repository
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop
```

Don't forget to source the ROS 2 environment:

```bash
source /opt/ros/humble/setup.bash
```

## Your First ROS 2 Node

Let's create a simple ROS 2 node in Python:

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

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

In this section, you've learned:
- The fundamental concepts of ROS 2
- How to install ROS 2
- How to create a basic publisher node

In the next section, we'll explore more advanced ROS 2 concepts and create subscriber nodes.
---
sidebar_position: 5
---

# Module 1 Projects

In this section, you'll work on hands-on projects that apply the concepts learned in this module.

## Project 1: Basic Robot Controller

### Objective
Create a simple robot controller that can move a simulated robot forward, backward, and rotate in place.

### Requirements
- Create a node that publishes Twist messages to control robot motion
- Implement a simple command interface (keyboard or service-based)
- Add safety features to prevent collisions

### Implementation Steps

1. **Create the publisher node**:
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Publisher for robot velocity commands
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Service for controlling robot movement
        self.move_forward_service = self.create_service(
            Trigger, 'move_forward', self.move_forward_callback
        )
        self.rotate_service = self.create_service(
            Trigger, 'rotate', self.rotate_callback
        )
        self.stop_service = self.create_service(
            Trigger, 'stop', self.stop_callback
        )

    def move_forward_callback(self, request, response):
        twist = Twist()
        twist.linear.x = 0.5  # Move forward at 0.5 m/s
        twist.angular.z = 0.0
        self.vel_publisher.publish(twist)

        response.success = True
        response.message = "Moving forward"
        return response

    def rotate_callback(self, request, response):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.5  # Rotate at 0.5 rad/s
        self.vel_publisher.publish(twist)

        response.success = True
        response.message = "Rotating"
        return response

    def stop_callback(self, request, response):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.vel_publisher.publish(twist)

        response.success = True
        response.message = "Stopped"
        return response

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

2. **Create a client node to call the services**:
```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class ControllerClient(Node):
    def __init__(self):
        super().__init__('controller_client')

        # Create clients for each service
        self.forward_client = self.create_client(Trigger, 'move_forward')
        self.rotate_client = self.create_client(Trigger, 'rotate')
        self.stop_client = self.create_client(Trigger, 'stop')

        # Wait for services to be available
        while not self.forward_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Forward service not available, waiting again...')

        while not self.rotate_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Rotate service not available, waiting again...')

        while not self.stop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Stop service not available, waiting again...')

    def move_forward(self):
        request = Trigger.Request()
        future = self.forward_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def rotate(self):
        request = Trigger.Request()
        future = self.rotate_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def stop(self):
        request = Trigger.Request()
        future = self.stop_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    client = ControllerClient()

    try:
        print("Robot Controller Client")
        print("Commands: 'f' - forward, 'r' - rotate, 's' - stop, 'q' - quit")

        while True:
            command = input("Enter command: ").strip().lower()

            if command == 'f':
                result = client.move_forward()
                print(f"Move forward result: {result.message}")
            elif command == 'r':
                result = client.rotate()
                print(f"Rotate result: {result.message}")
            elif command == 's':
                result = client.stop()
                print(f"Stop result: {result.message}")
            elif command == 'q':
                break
            else:
                print("Invalid command. Use 'f', 'r', 's', or 'q'.")

    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Testing
1. Run the controller node: `ros2 run your_package robot_controller`
2. In another terminal, run the client: `ros2 run your_package controller_client`
3. Test the different commands to control the robot

## Project 2: Sensor Data Aggregator

### Objective
Create a system that subscribes to multiple sensor topics, processes the data, and publishes aggregated results.

### Requirements
- Subscribe to at least 3 different sensor topics
- Process and combine the sensor data
- Publish processed results to a new topic
- Implement error handling for missing sensors

### Implementation Steps

1. **Create the aggregator node**:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range, BatteryState
from std_msgs.msg import Float32MultiArray

class SensorAggregator(Node):
    def __init__(self):
        super().__init__('sensor_aggregator')

        # Subscriptions for different sensors
        self.scan_subscription = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10
        )
        self.range_subscription = self.create_subscription(
            Range, 'range', self.range_callback, 10
        )
        self.battery_subscription = self.create_subscription(
            BatteryState, 'battery', self.battery_callback, 10
        )

        # Publisher for aggregated data
        self.aggregated_publisher = self.create_publisher(
            Float32MultiArray, 'aggregated_sensors', 10
        )

        # Storage for sensor data
        self.scan_data = None
        self.range_data = None
        self.battery_data = None

        # Timer to publish aggregated data periodically
        self.timer = self.create_timer(0.1, self.publish_aggregated_data)  # 10 Hz

    def scan_callback(self, msg):
        # Store laser scan data (first few readings as example)
        self.scan_data = list(msg.ranges[:10]) if msg.ranges else []

    def range_callback(self, msg):
        # Store range finder data
        self.range_data = msg.range

    def battery_callback(self, msg):
        # Store battery data
        self.battery_data = msg.percentage

    def publish_aggregated_data(self):
        # Check if we have data from all sensors
        if self.scan_data is not None and self.range_data is not None and self.battery_data is not None:
            # Create aggregated message
            aggregated_msg = Float32MultiArray()

            # Combine all sensor data into one array
            combined_data = [
                float(len(self.scan_data)),  # Number of scan readings
                self.range_data,             # Range reading
                self.battery_data            # Battery percentage
            ]
            combined_data.extend(self.scan_data[:5])  # First 5 scan readings

            aggregated_msg.data = combined_data
            self.aggregated_publisher.publish(aggregated_msg)

            self.get_logger().info(f'Published aggregated data: {combined_data}')

def main(args=None):
    rclpy.init(args=args)
    aggregator = SensorAggregator()

    try:
        rclpy.spin(aggregator)
    except KeyboardInterrupt:
        pass
    finally:
        aggregator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Testing
1. Run the aggregator: `ros2 run your_package sensor_aggregator`
2. Simulate or connect to sensor data sources
3. Monitor the aggregated output: `ros2 topic echo /aggregated_sensors std_msgs/msg/Float32MultiArray`

## Project 3: Parameter-Based Navigation System

### Objective
Create a navigation system that adjusts its behavior based on runtime parameters.

### Requirements
- Use parameters to configure navigation behavior
- Implement dynamic parameter callback
- Create a navigation algorithm that responds to parameter changes
- Log parameter changes for debugging

### Implementation Steps

1. **Create the parameter-based navigator**:
```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math

class ParameterNavigator(Node):
    def __init__(self):
        super().__init__('parameter_navigator')

        # Declare parameters with default values
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('safety_distance', 0.5)
        self.declare_parameter('target_tolerance', 0.1)

        # Initialize parameter values
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.safety_distance = self.get_parameter('safety_distance').value
        self.target_tolerance = self.get_parameter('target_tolerance').value

        # Publisher for velocity commands
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriptions for odometry and laser scan
        self.odom_subscription = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10
        )

        # Timer for navigation loop
        self.nav_timer = self.create_timer(0.1, self.navigation_loop)

        # Current robot pose
        self.current_pose = None

        # Register parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def navigation_loop(self):
        if self.current_pose is None:
            return  # Wait for initial pose

        # Simple navigation logic (placeholder)
        cmd_vel = Twist()

        # Adjust speeds based on parameters
        cmd_vel.linear.x = min(0.2, self.max_linear_speed)  # Simple forward movement
        cmd_vel.angular.z = 0.0  # No rotation in this example

        # Safety check based on parameters
        # (In a real system, you'd check sensor data here)
        if True:  # Placeholder for safety check
            self.vel_publisher.publish(cmd_vel)
        else:
            # Stop if safety condition is violated
            stop_cmd = Twist()
            self.vel_publisher.publish(stop_cmd)

    def parameter_callback(self, params):
        """Callback for parameter changes."""
        for param in params:
            if param.name == 'max_linear_speed':
                self.max_linear_speed = param.value
                self.get_logger().info(f'Max linear speed updated to: {param.value}')
            elif param.name == 'max_angular_speed':
                self.max_angular_speed = param.value
                self.get_logger().info(f'Max angular speed updated to: {param.value}')
            elif param.name == 'safety_distance':
                self.safety_distance = param.value
                self.get_logger().info(f'Safety distance updated to: {param.value}')
            elif param.name == 'target_tolerance':
                self.target_tolerance = param.value
                self.get_logger().info(f'Target tolerance updated to: {param.value}')

        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    navigator = ParameterNavigator()

    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Testing
1. Run the navigator: `ros2 run your_package parameter_navigator`
2. Change parameters dynamically: `ros2 param set /parameter_navigator max_linear_speed 1.0`
3. Observe how the system responds to parameter changes

## Summary

These projects helped you practice:
- Creating nodes with publishers and services
- Working with different message types
- Implementing parameter-based systems
- Combining multiple ROS concepts in real applications

Complete these projects to solidify your understanding of ROS 2 fundamentals before moving to Module 2.
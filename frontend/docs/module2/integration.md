---
sidebar_position: 4
---

# Integrating Simulation with ROS 2

In this section, we'll explore how to tightly integrate Gazebo simulation with your ROS 2 nodes from Module 1, creating a seamless development workflow.

## ROS 2 - Gazebo Bridge

The `ros_gz_bridge` package allows bidirectional communication between ROS 2 and Gazebo. This bridge translates ROS 2 messages to Gazebo messages and vice versa.

### Installation

```bash
# Install the bridge package
sudo apt install ros-humble-ros-gz-bridge
```

### Running the Bridge

```bash
# Run the bridge with a specific configuration
ros2 run ros_gz_bridge parameter_bridge \
  --ros-args -p topic_name:=/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist \
  -p topic_name2:=/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry
```

## Creating a Complete Simulation Package

Let's create a complete ROS 2 package that integrates with Gazebo:

### Package Structure
```
robot_simulation/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── robot_world.launch.py
│   └── bringup_launch.py
├── models/
│   └── my_robot/
│       ├── model.sdf
│       └── meshes/
├── worlds/
│   ├── simple_room.world
│   └── maze.world
├── rviz/
│   └── robot_view.rviz
└── config/
    └── robot_control.yaml
```

### Launch File Example

```python
# launch/robot_world.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='simple_room.world')

    # Paths
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')
    pkg_robot_simulation = FindPackageShare('robot_simulation')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([pkg_robot_simulation, 'worlds', world]),
            'verbose': 'false',
        }.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Spawn Entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='simple_room.world',
            description='Choose one of the world files from `/robot_simulation/worlds`'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])
```

## Advanced Simulation Control

### Dynamic Reconfiguration

You can dynamically change simulation parameters using ROS 2 services:

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

class SimulationController(Node):
    def __init__(self):
        super().__init__('simulation_controller')

        # Service to pause/resume simulation
        self.pause_service = self.create_service(
            SetBool, 'pause_simulation', self.pause_simulation_callback
        )

        # Service to reset simulation
        self.reset_service = self.create_service(
            SetBool, 'reset_simulation', self.reset_simulation_callback
        )

        # Publisher to Gazebo world control topic
        self.world_control_pub = self.create_publisher(
            # This would use gz.msgs.WorldControl in a real implementation
            # For this example, we'll simulate with a custom message
            String,
            '/world_control',
            10
        )

    def pause_simulation_callback(self, request, response):
        """Pause or resume the simulation."""
        if request.data:
            # Send pause command to Gazebo
            cmd_msg = String()
            cmd_msg.data = "pause"
            self.world_control_pub.publish(cmd_msg)
            response.success = True
            response.message = "Simulation paused"
        else:
            # Send unpause command to Gazebo
            cmd_msg = String()
            cmd_msg.data = "unpause"
            self.world_control_pub.publish(cmd_msg)
            response.success = True
            response.message = "Simulation resumed"

        return response

    def reset_simulation_callback(self, request, response):
        """Reset the simulation."""
        if request.data:  # Reset requested
            cmd_msg = String()
            cmd_msg.data = "reset"
            self.world_control_pub.publish(cmd_msg)
            response.success = True
            response.message = "Simulation reset"
        else:
            response.success = False
            response.message = "Reset cancelled"

        return response

def main(args=None):
    rclpy.init(args=args)
    controller = SimulationController()

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

## Sensor Data Processing

Simulated sensors provide data that should match real sensors as closely as possible:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2

class SensorProcessor(Node):
    def __init__(self):
        super().__init__('sensor_processor')

        # Create CV bridge for image processing
        self.cv_bridge = CvBridge()

        # Subscriptions for different sensor types
        self.laser_sub = self.create_subscription(
            LaserScan, '/laser_scan', self.laser_callback, 10
        )
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10
        )

        # Publishers for processed data
        self.obstacle_pub = self.create_publisher(
            Bool, '/obstacle_detected', 10
        )
        self.processed_image_pub = self.create_publisher(
            Image, '/processed_image', 10
        )

        # Store camera info
        self.camera_info = None

    def laser_callback(self, msg):
        """Process laser scan data to detect obstacles."""
        # Find minimum distance in forward arc (±30 degrees)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        # Calculate indices for forward arc
        forward_start_idx = int((np.radians(-30) - angle_min) / angle_increment)
        forward_end_idx = int((np.radians(30) - angle_min) / angle_increment)

        # Make sure indices are within bounds
        forward_start_idx = max(0, forward_start_idx)
        forward_end_idx = min(len(msg.ranges), forward_end_idx)

        # Extract forward ranges
        forward_ranges = msg.ranges[forward_start_idx:forward_end_idx]

        # Filter out invalid ranges (inf, nan)
        valid_ranges = [r for r in forward_ranges if r != float('inf') and not np.isnan(r)]

        if valid_ranges:
            min_distance = min(valid_ranges)

            # Check if obstacle is closer than safety threshold
            safety_threshold = 0.5  # meters
            obstacle_msg = Bool()
            obstacle_msg.data = min_distance < safety_threshold

            self.obstacle_pub.publish(obstacle_msg)

            self.get_logger().info(f'Closest obstacle: {min_distance:.2f}m, Obstacle: {obstacle_msg.data}')

    def camera_callback(self, msg):
        """Process camera image."""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Example: Simple edge detection
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)

            # Convert back to ROS Image message
            processed_msg = self.cv_bridge.cv2_to_imgmsg(edges, encoding='mono8')
            processed_msg.header = msg.header  # Preserve header info

            self.processed_image_pub.publish(processed_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {str(e)}')

    def camera_info_callback(self, msg):
        """Store camera calibration info."""
        self.camera_info = msg

def main(args=None):
    rclpy.init(args=args)
    processor = SensorProcessor()

    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Simulation Validation

Validate that your simulation behaves like the real world:

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class SimulationValidator(Node):
    def __init__(self):
        super().__init__('simulation_validator')

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Store previous state for validation
        self.prev_odom = None
        self.velocity_errors = []

        # Timer for sending validation commands
        self.timer = self.create_timer(1.0, self.send_validation_commands)

    def odom_callback(self, msg):
        """Validate odometry against expected motion."""
        if self.prev_odom is not None:
            # Calculate time difference
            dt = (msg.header.stamp.sec - self.prev_odom.header.stamp.sec) + \
                 (msg.header.stamp.nanosec - self.prev_odom.header.stamp.nanosec) / 1e9

            if dt > 0:
                # Calculate actual velocities from odometry
                dx = msg.pose.pose.position.x - self.prev_odom.pose.pose.position.x
                dy = msg.pose.pose.position.y - self.prev_odom.pose.pose.position.y
                dz = msg.pose.pose.position.z - self.prev_odom.pose.pose.position.z

                actual_vx = math.sqrt(dx*dx + dy*dy) / dt
                actual_vz = math.sqrt(dz*dz) / dt  # Simplified for example

                # Compare with expected velocities (stored from commands)
                # This would require storing the commanded velocities
                # For this example, we'll just log the actual velocities

                self.get_logger().info(f'Actual velocity: vx={actual_vx:.2f}, vz={actual_vz:.2f}')

        self.prev_odom = msg

    def send_validation_commands(self):
        """Send commands to test robot motion."""
        cmd = Twist()
        cmd.linear.x = 0.5  # Move forward at 0.5 m/s
        cmd.angular.z = 0.2  # Turn at 0.2 rad/s
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    validator = SimulationValidator()

    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        # Print validation statistics
        print(f"Velocity errors recorded: {len(validator.velocity_errors)}")
    finally:
        validator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices for Simulation Integration

1. **Realistic Parameters**: Use realistic physical parameters for your robot models
2. **Sensor Noise**: Add appropriate noise models to simulate real sensor behavior
3. **Latency Simulation**: Account for communication delays in real systems
4. **Computational Load**: Simulate computational constraints that affect real-time performance
5. **Validation**: Regularly compare simulation results with real-world tests

## Exercise

Create a complete simulation scenario that:
1. Spawns a robot in a Gazebo world
2. Implements a simple navigation task (go to goal)
3. Uses sensor data for obstacle avoidance
4. Validates the robot's motion against expected behavior
5. Logs simulation metrics for analysis

## Summary

In this section, you learned:
- How to create complete simulation packages
- How to integrate ROS 2 nodes with Gazebo
- How to process and validate sensor data in simulation
- Best practices for realistic simulation
- Techniques for simulation validation

In the next section, we'll explore advanced simulation techniques and comparison with real hardware.
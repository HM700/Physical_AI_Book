---
sidebar_position: 2
---

# Gazebo Simulation Environment

Gazebo is a 3D dynamic simulator that supports accurate simulation of robots in complex indoor and outdoor environments. It's widely used in robotics research and development.

## Introduction to Gazebo

Gazebo simulates multiple robots in a 3D environment with realistic physics properties. It includes:
- High-quality graphics rendering
- Accurate physics simulation
- Sensors simulation (cameras, lidars, etc.)
- Plugins for extending functionality

## Installing Gazebo

For this course, we'll use Gazebo Garden:

```bash
# On Ubuntu 22.04
sudo apt update
sudo apt install gazebo libgazebo-dev

# Or install the full desktop version
sudo apt install gazebo libgazebo-dev gazebo-tools
```

## Basic Gazebo Concepts

### Worlds
World files define the environment in which robots operate. They contain:
- Terrain and environment models
- Lighting conditions
- Physics properties
- Initial robot positions

Example world file (`my_world.sdf`):
```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="my_world">
    <!-- Include a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include a sun light -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Define a simple box obstacle -->
    <model name="box">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Running Gazebo

Start Gazebo with a specific world:
```bash
gazebo my_world.sdf
```

Or start with an empty world:
```bash
gazebo
```

## Integrating with ROS 2

Gazebo provides bridges to ROS 2 through the `ros_gz_bridge` package, allowing communication between Gazebo and ROS 2 nodes.

### Launching a Robot in Gazebo

Example launch file to spawn a robot:
```python
# launch/robot_spawn.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Get the launch directory
    pkg_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

    return LaunchDescription([
        # Launch Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Spawn robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'my_robot',
                '-file', os.path.join(pkg_dir, 'models', 'my_robot.urdf'),
                '-x', '0', '-y', '0', '-z', '0.5'
            ],
            output='screen'
        )
    ])
```

## Robot Modeling with URDF

URDF (Unified Robot Description Format) describes robot models in XML format. Here's an example differential drive robot:

```xml
<?xml version="1.0"?>
<robot name="simple_diff_drive_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Left wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <origin rpy="1.5708 0 0"/>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <origin rpy="1.5708 0 0"/>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Right wheel -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <origin rpy="1.5708 0 0"/>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <origin rpy="1.5708 0 0"/>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="-0.15 0.15 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="-0.15 -0.15 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Differential drive controller plugin -->
  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.3</wheel_separation>
      <wheel_diameter>0.2</wheel_diameter>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>
  </gazebo>
</robot>
```

## Controlling Robots in Gazebo

Once your robot is in Gazebo, you can control it with ROS 2 topics:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class GazeboController(Node):
    def __init__(self):
        super().__init__('gazebo_controller')

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer to send commands periodically
        self.timer = self.create_timer(0.1, self.send_command)
        self.i = 0

    def send_command(self):
        twist = Twist()

        # Alternate between forward and rotation
        if self.i % 20 < 10:
            twist.linear.x = 0.5  # Move forward
            twist.angular.z = 0.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.5  # Rotate

        self.cmd_vel_pub.publish(twist)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    controller = GazeboController()

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

## Adding Sensors to Robots

Example URDF with a camera sensor:

```xml
<!-- Add to your robot URDF -->
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
    <material name="red">
      <color rgba="1 0 0 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.01"/>
    <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
  </inertial>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.2 0 0.05" rpy="0 0 0"/>
</joint>

<gazebo reference="camera_link">
  <sensor type="camera" name="camera_sensor">
    <update_rate>30</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
      <frame_name>camera_link</frame_name>
      <topic_name>camera/image_raw</topic_name>
    </plugin>
  </sensor>
</gazebo>
```

## Best Practices

- **Physics Accuracy**: Tune your robot's inertial properties for realistic simulation
- **Sensor Placement**: Position sensors thoughtfully to match real-world placement
- **Environment Complexity**: Start with simple environments and increase complexity gradually
- **Performance**: Balance visual quality with simulation speed
- **Validation**: Compare simulation results with real-world behavior when possible

## Exercise

Create a simple robot model with:
1. A base and two wheels
2. A differential drive controller plugin
3. A camera sensor
4. A launch file to spawn the robot in Gazebo
5. A ROS 2 node to control the robot's movement

## Summary

In this section, you learned:
- How to set up and use Gazebo simulation
- How to create robot models in URDF format
- How to integrate Gazebo with ROS 2
- How to add sensors to robot models
- Best practices for effective simulation

In the next section, we'll explore Unity integration for advanced simulation.
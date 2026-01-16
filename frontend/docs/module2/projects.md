---
sidebar_position: 5
---

# Module 2 Projects

In this section, you'll work on hands-on projects that apply the concepts learned about digital twins and simulation environments.

## Project 1: Complete Robot Simulation Environment

### Objective
Create a complete simulation environment with a robot, sensors, and a complex world that integrates with ROS 2.

### Requirements
- Create a robot model with differential drive
- Implement realistic physics properties
- Add multiple sensors (camera, lidar, IMU)
- Create a complex environment with obstacles
- Integrate with ROS 2 using the Gazebo bridge
- Implement basic navigation in simulation

### Implementation Steps

1. **Create the robot URDF** (`models/diff_robot.urdf`):
```xml
<?xml version="1.0"?>
<robot name="diff_robot">
  <!-- Base link -->
  <link name="base_footprint"/>

  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.15"/>
      </geometry>
      <material name="orange">
        <color rgba="1 0.5 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="0.416666" ixy="0" ixz="0" iyy="0.766666" iyz="0" izz="1.166666"/>
    </inertial>
  </link>

  <!-- Wheels -->
  <link name="wheel_left">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <origin rpy="1.5708 0 0"/>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <origin rpy="1.5708 0 0"/>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <link name="wheel_right">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <origin rpy="1.5708 0 0"/>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <origin rpy="1.5708 0 0"/>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="wheel_left_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left"/>
    <origin xyz="0 0.2 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="wheel_right_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_right"/>
    <origin xyz="0 -0.2 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Castor wheel -->
  <link name="caster_wheel">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="0.2 0 -0.1"/>
  </joint>

  <!-- Sensors -->
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

  <!-- Gazebo plugins -->
  <gazebo reference="base_link">
    <material>Gazebo/Orange</material>
  </gazebo>

  <gazebo reference="wheel_left">
    <mu1>10</mu1>
    <mu2>10</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
    <material>Gazebo/Black</material>
    <turnGravityOff>false</turnGravityOff>
  </gazebo>

  <gazebo reference="wheel_right">
    <mu1>10</mu1>
    <mu2>10</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
    <material>Gazebo/Black</material>
    <turnGravityOff>false</turnGravityOff>
  </gazebo>

  <gazebo reference="caster_wheel">
    <mu1>1</mu1>
    <mu2>1</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
    <material>Gazebo/Gray</material>
    <turnGravityOff>false</turnGravityOff>
  </gazebo>

  <!-- Differential drive plugin -->
  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <left_joint>wheel_left_joint</left_joint>
      <right_joint>wheel_right_joint</right_joint>
      <wheel_separation>0.4</wheel_separation>
      <wheel_diameter>0.2</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_footprint</robot_base_frame>
      <publish_odom>true</publish_odom>
      <publish_wheel_tf>true</publish_wheel_tf>
      <publish_odom_tf>true</publish_odom_tf>
      <legacy_mode>false</legacy_mode>
      <update_rate>30</update_rate>
    </plugin>
  </gazebo>

  <!-- Camera plugin -->
  <gazebo reference="camera_link">
    <sensor type="camera" name="camera1">
      <update_rate>30.0</update_rate>
      <camera name="head">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <frame_name>camera_link</frame_name>
        <topic_name>camera/image_raw</topic_name>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

2. **Create a complex world file** (`worlds/complex_world.sdf`):
```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="complex_world">
    <!-- Physics -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Walls -->
    <model name="wall_1">
      <pose>0 -5 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>20 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>20 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="wall_2">
      <pose>0 5 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>20 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>20 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="wall_3">
      <pose>-10 0 0.5 0 0 1.5708</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="wall_4">
      <pose>10 0 0.5 0 0 1.5708</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Obstacles -->
    <model name="obstacle_1">
      <pose>3 2 0.5 0 0 0</pose>
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
            <ambient>0.5 0 0 1</ambient>
            <diffuse>0.5 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="obstacle_2">
      <pose>-3 -2 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>1</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0 0.5 0 1</ambient>
            <diffuse>0 0.5 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Goal marker -->
    <model name="goal">
      <pose>8 0 0.1 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>0.2</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0 1 0 0.5</ambient>
            <diffuse>0 1 0 0.5</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

3. **Create a launch file** (`launch/sim_robot.launch.py`):
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Package locations
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')
    pkg_sim = FindPackageShare('your_simulation_package')  # Replace with your package name

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Launch Gazebo with world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'empty_world.launch.py'])
            ]),
            launch_arguments={
                'world': PathJoinSubstitution([FindPackageShare('your_simulation_package'), 'worlds', 'complex_world.sdf']),
                'gui': 'true',
                'verbose': 'false',
            }.items()
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[PathJoinSubstitution([FindPackageShare('your_simulation_package'), 'models', 'diff_robot.urdf'])]
        ),

        # Spawn Robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'diff_robot',
                '-file', PathJoinSubstitution([FindPackageShare('your_simulation_package'), 'models', 'diff_robot.urdf']),
                '-x', '0',
                '-y', '0',
                '-z', '0.1'
            ],
            output='screen'
        ),

        # RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('your_simulation_package'), 'rviz', 'config.rviz'])],
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])
```

4. **Create a navigation controller**:
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10
        )

        # Navigation goals
        self.goals = [
            (5.0, 0.0),   # Goal 1: x=5, y=0
            (5.0, 3.0),   # Goal 2: x=5, y=3
            (-2.0, -3.0), # Goal 3: x=-2, y=-3
            (0.0, 0.0)    # Goal 4: Return to start
        ]

        self.current_goal_index = 0
        self.current_pose = None
        self.arrival_tolerance = 0.5  # meters

        # Control parameters
        self.linear_speed = 0.5
        self.angular_speed = 0.5
        self.rotation_threshold = 0.1  # radians

        # Timer for navigation loop
        self.nav_timer = self.create_timer(0.1, self.navigation_loop)

    def odom_callback(self, msg):
        """Update current pose from odometry."""
        self.current_pose = msg.pose.pose

    def calculate_distance(self, pos1, pos2):
        """Calculate Euclidean distance between two positions."""
        return math.sqrt((pos1.x - pos2[0])**2 + (pos1.y - pos2[1])**2)

    def calculate_angle(self, pos1, pos2):
        """Calculate angle between current pose and target."""
        dx = pos2[0] - pos1.x
        dy = pos2[1] - pos1.y
        return math.atan2(dy, dx)

    def navigation_loop(self):
        """Main navigation control loop."""
        if self.current_pose is None:
            return

        # Get current position
        current_pos = (
            self.current_pose.position.x,
            self.current_pose.position.y
        )

        # Get current orientation
        orientation_q = self.current_pose.orientation
        _, _, current_yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])

        # Get current goal
        current_goal = self.goals[self.current_goal_index]

        # Calculate distance to goal
        distance_to_goal = self.calculate_distance(self.current_pose.position, current_goal)

        # Check if reached current goal
        if distance_to_goal < self.arrival_tolerance:
            self.get_logger().info(f'Goal {self.current_goal_index + 1} reached: {current_goal}')

            # Move to next goal
            self.current_goal_index = (self.current_goal_index + 1) % len(self.goals)
            current_goal = self.goals[self.current_goal_index]
            self.get_logger().info(f'Moving to goal {self.current_goal_index + 1}: {current_goal}')

        # Calculate desired angle to goal
        desired_angle = self.calculate_angle(self.current_pose.position, current_goal)

        # Calculate angle difference
        angle_diff = desired_angle - current_yaw

        # Normalize angle to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # Create velocity command
        cmd_vel = Twist()

        # If angle is too large, rotate first
        if abs(angle_diff) > self.rotation_threshold:
            cmd_vel.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
        else:
            # Move toward goal
            cmd_vel.linear.x = self.linear_speed
            cmd_vel.angular.z = self.angular_speed * angle_diff  # Proportional control

        # Publish command
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    controller = NavigationController()

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

### Testing the Project
1. Launch the simulation: `ros2 launch your_package sim_robot.launch.py`
2. In another terminal, run the navigation controller: `ros2 run your_package navigation_controller.py`
3. Monitor the robot's progress in Gazebo and RViz

## Project 2: Unity-Gazebo Comparison Study

### Objective
Compare the behavior of the same robot model in both Gazebo and Unity simulations to understand the differences and similarities.

### Requirements
- Create identical robot models in both simulators
- Run the same control algorithm in both
- Record and compare performance metrics
- Analyze differences in physics simulation
- Document findings

## Project 3: Multi-Robot Simulation

### Objective
Extend the simulation to include multiple robots that can coordinate with each other.

### Requirements
- Spawn multiple instances of the robot model
- Implement robot-to-robot communication
- Coordinate movement between robots
- Avoid collisions between robots
- Demonstrate formation control or other coordination behaviors

## Summary

These projects helped you practice:
- Creating complex robot models with multiple sensors
- Building rich simulation environments
- Implementing navigation algorithms in simulation
- Comparing different simulation platforms
- Working with multi-robot systems

Complete these projects to solidify your understanding of digital twin simulation before moving to Module 3.
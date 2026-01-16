---
sidebar_position: 5
---

# Module 3 Projects

In this section, you'll work on hands-on projects that apply the concepts learned about AI-Robot brains, Isaac Sim, and perception/navigation systems.

## Project 1: Complete AI-Robot Brain System

### Objective
Create a complete AI-Robot brain system that integrates perception, navigation, and decision-making in Isaac Sim.

### Requirements
- Create a robot with multiple sensors (RGB-D camera, IMU, wheel encoders)
- Implement perception pipeline with object detection and semantic segmentation
- Implement navigation system with path planning and obstacle avoidance
- Create decision-making system for task execution
- Integrate with Isaac Sim for photorealistic simulation

### Implementation Steps

1. **Create the robot configuration** (`config/ai_robot.urdf`):
```xml
<?xml version="1.0"?>
<robot name="ai_robot">
  <!-- Base link -->
  <link name="base_footprint">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
  </joint>

  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.3" length="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.3" length="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="20.0"/>
      <inertia ixx="0.416666" ixy="0" ixz="0" iyy="0.416666" iyz="0" izz="0.9"/>
    </inertial>
  </link>

  <!-- Wheels -->
  <link name="wheel_fl">
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
      <mass value="1.0"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <link name="wheel_fr">
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
      <mass value="1.0"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <link name="wheel_bl">
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
      <mass value="1.0"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <link name="wheel_br">
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
      <mass value="1.0"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="wheel_fl_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_fl"/>
    <origin xyz="0.2 0.2 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="wheel_fr_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_fr"/>
    <origin xyz="0.2 -0.2 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="wheel_bl_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_bl"/>
    <origin xyz="-0.2 0.2 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="wheel_br_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_br"/>
    <origin xyz="-0.2 -0.2 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Sensors -->
  <link name="imu_link">
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
    </inertial>
  </link>

  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
    </inertial>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.25 0 0.05" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo plugins -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="wheel_fl">
    <mu1>10.0</mu1>
    <mu2>10.0</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
    <material>Gazebo/Black</material>
  </gazebo>

  <gazebo reference="wheel_fr">
    <mu1>10.0</mu1>
    <mu2>10.0</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
    <material>Gazebo/Black</material>
  </gazebo>

  <gazebo reference="wheel_bl">
    <mu1>10.0</mu1>
    <mu2>10.0</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
    <material>Gazebo/Black</material>
  </gazebo>

  <gazebo reference="wheel_br">
    <mu1>10.0</mu1>
    <mu2>10.0</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
    <material>Gazebo/Black</material>
  </gazebo>

  <!-- Differential drive plugin -->
  <gazebo>
    <plugin name="drive_controller" filename="libgazebo_ros_diff_drive.so">
      <left_joint>wheel_fl_joint</left_joint>
      <right_joint>wheel_fr_joint</right_joint>
      <wheel_separation>0.4</wheel_separation>
      <wheel_diameter>0.2</wheel_diameter>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_footprint</robot_base_frame>
      <publish_odom>true</publish_odom>
      <publish_wheel_tf>true</publish_wheel_tf>
      <publish_odom_tf>true</publish_odom_tf>
      <legacy_mode>false</legacy_mode>
    </plugin>
  </gazebo>

  <!-- IMU Sensor -->
  <gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>true</visualize>
      <imu>
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </z>
        </angular_velocity>
        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </z>
        </linear_acceleration>
      </imu>
    </sensor>
  </gazebo>

  <!-- Camera Sensor -->
  <gazebo reference="camera_link">
    <sensor name="camera_sensor" type="camera">
      <update_rate>30</update_rate>
      <camera name="head_camera">
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
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <frame_name>camera_link</frame_name>
        <topic_name>camera/image_raw</topic_name>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

2. **Create the AI-Robot Brain Node**:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import numpy as np
import tensorflow as tf
from collections import deque
import threading

class AIRobotBrain(Node):
    def __init__(self):
        super().__init__('ai_robot_brain')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.detection_sub = self.create_subscription(
            Detection2DArray, '/detections', self.detection_callback, 10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.behavior_pub = self.create_publisher(String, '/behavior_status', 10)

        # Robot state
        self.current_pose = None
        self.current_twist = None
        self.imu_data = None
        self.scan_data = None
        self.last_image = None
        self.detections = []

        # AI brain components
        self.perception_system = PerceptionSystem()
        self.navigation_system = NavigationSystem()
        self.decision_maker = DecisionMaker()

        # Task queue
        self.task_queue = deque()
        self.current_task = None

        # State machine
        self.brain_state = 'IDLE'  # IDLE, PERCEIVING, NAVIGATING, EXECUTING, WAITING

        # Timers
        self.brain_timer = self.create_timer(0.1, self.brain_loop)

        # Threading for heavy computations
        self.lock = threading.Lock()

    def image_callback(self, msg):
        """Process camera images for perception."""
        with self.lock:
            self.last_image = msg

    def imu_callback(self, msg):
        """Process IMU data."""
        with self.lock:
            self.imu_data = msg

    def odom_callback(self, msg):
        """Process odometry data."""
        with self.lock:
            self.current_pose = msg.pose.pose
            self.current_twist = msg.twist.twist

    def scan_callback(self, msg):
        """Process laser scan data."""
        with self.lock:
            self.scan_data = msg

    def detection_callback(self, msg):
        """Process object detections."""
        with self.lock:
            self.detections = msg.detections

    def brain_loop(self):
        """Main AI brain state machine."""
        with self.lock:
            if self.brain_state == 'IDLE':
                # Check for tasks in queue
                if self.task_queue:
                    self.current_task = self.task_queue.popleft()
                    self.brain_state = 'PERCEIVING'
                    self.get_logger().info(f'Starting task: {self.current_task}')

            elif self.brain_state == 'PERCEIVING':
                # Process sensor data
                if self.last_image:
                    # Run perception pipeline
                    perception_result = self.perception_system.process(
                        self.last_image,
                        self.scan_data,
                        self.imu_data
                    )

                    # Update navigation system with perception
                    self.navigation_system.update_environment(perception_result)

                    # Decide next action
                    next_action = self.decision_maker.decide_action(
                        perception_result,
                        self.current_task,
                        self.current_pose
                    )

                    if next_action['type'] == 'navigate':
                        self.navigation_system.set_goal(next_action['goal'])
                        self.brain_state = 'NAVIGATING'
                    elif next_action['type'] == 'execute':
                        self.brain_state = 'EXECUTING'
                        self.execute_action(next_action['action'])

            elif self.brain_state == 'NAVIGATING':
                # Execute navigation
                nav_status = self.navigation_system.update(
                    self.current_pose,
                    self.current_twist
                )

                if nav_status == 'arrived':
                    self.brain_state = 'EXECUTING'
                elif nav_status == 'failed':
                    self.brain_state = 'WAITING'  # Need human intervention

            elif self.brain_state == 'EXECUTING':
                # Execute the planned action
                exec_status = self.execute_current_action()

                if exec_status == 'completed':
                    self.brain_state = 'IDLE'
                    self.publish_behavior_status('Task completed')
                elif exec_status == 'failed':
                    self.brain_state = 'WAITING'

            elif self.brain_state == 'WAITING':
                # Wait for external command or timeout
                pass

    def execute_action(self, action):
        """Execute a specific action."""
        if action['type'] == 'pick_object':
            return self.execute_pick_action(action)
        elif action['type'] == 'place_object':
            return self.execute_place_action(action)
        elif action['type'] == 'inspect_area':
            return self.execute_inspect_action(action)
        else:
            self.get_logger().warn(f'Unknown action type: {action["type"]}')
            return 'failed'

    def execute_current_action(self):
        """Execute the current action."""
        # This would be implemented based on the specific action
        # For now, return completed after some time
        return 'completed'

    def add_task(self, task):
        """Add a task to the queue."""
        self.task_queue.append(task)

    def publish_behavior_status(self, status):
        """Publish behavior status."""
        status_msg = String()
        status_msg.data = status
        self.behavior_pub.publish(status_msg)

class PerceptionSystem:
    """Handles all perception tasks."""
    def __init__(self):
        # Initialize perception models
        self.object_detector = None  # Would be loaded model
        self.segmentation_model = None  # Would be loaded model
        self.depth_estimator = None  # Would be loaded model

    def process(self, image_msg, scan_data, imu_data):
        """Process all sensor data for perception."""
        # Convert image
        cv_image = CvBridge().imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

        # Run object detection
        objects = self.detect_objects(cv_image)

        # Run semantic segmentation
        segmentation = self.semantic_segment(cv_image)

        # Process depth from stereo or structure from motion
        depth_map = self.estimate_depth(cv_image)

        # Fuse with laser and IMU data
        environment_map = self.fuse_sensor_data(
            objects, segmentation, depth_map, scan_data, imu_data
        )

        return {
            'objects': objects,
            'segmentation': segmentation,
            'environment_map': environment_map,
            'pose_estimate': self.estimate_pose(imu_data)
        }

    def detect_objects(self, image):
        """Detect objects in the image."""
        # Would use trained model
        return []

    def semantic_segment(self, image):
        """Perform semantic segmentation."""
        # Would use trained model
        return []

    def estimate_depth(self, image):
        """Estimate depth from image."""
        # Would use stereo or monocular depth estimation
        return []

    def fuse_sensor_data(self, objects, segmentation, depth_map, scan_data, imu_data):
        """Fuse all sensor data into coherent environment representation."""
        # Implementation would combine all modalities
        return {}

    def estimate_pose(self, imu_data):
        """Estimate pose from IMU data."""
        return {'position': [0, 0, 0], 'orientation': [0, 0, 0, 1]}

class NavigationSystem:
    """Handles navigation and path planning."""
    def __init__(self):
        self.goal = None
        self.current_path = []
        self.local_planner = None  # DWA or other local planner
        self.global_planner = None  # A* or other global planner
        self.environment_map = {}

    def set_goal(self, goal):
        """Set navigation goal."""
        self.goal = goal
        self.plan_path()

    def update_environment(self, perception_result):
        """Update environment map with new perception data."""
        self.environment_map = perception_result['environment_map']

    def plan_path(self):
        """Plan path to goal."""
        if self.goal and self.environment_map:
            # Call global planner
            self.current_path = self.global_planner.plan(self.goal, self.environment_map)

    def update(self, current_pose, current_twist):
        """Update navigation state."""
        if not self.current_path:
            return 'arrived'

        # Execute local planning and control
        cmd_vel = self.local_planner.compute_velocity(
            current_pose, current_twist, self.current_path
        )

        # Publish command
        # Would publish to cmd_vel topic

        # Check arrival
        if self.is_at_goal(current_pose):
            return 'arrived'

        return 'navigating'

    def is_at_goal(self, pose):
        """Check if robot is at goal."""
        # Implementation would check distance to goal
        return False

class DecisionMaker:
    """Makes high-level decisions based on perception and tasks."""
    def __init__(self):
        self.task_memory = {}  # Remember task outcomes
        self.environment_model = {}  # Model of environment state

    def decide_action(self, perception_result, current_task, current_pose):
        """Decide next action based on perception and task."""
        if current_task['type'] == 'explore':
            return self.decide_explore_action(perception_result, current_task)
        elif current_task['type'] == 'find_object':
            return self.decide_find_object_action(perception_result, current_task)
        elif current_task['type'] == 'navigate_to':
            return self.decide_navigate_action(current_task)
        else:
            return {'type': 'wait', 'reason': 'unknown_task'}

    def decide_explore_action(self, perception_result, task):
        """Decide action for exploration task."""
        # Find unexplored areas
        unexplored = self.find_unexplored_areas(perception_result['environment_map'])
        if unexplored:
            return {'type': 'navigate', 'goal': unexplored[0]}
        else:
            return {'type': 'complete', 'reason': 'area_explored'}

    def decide_find_object_action(self, perception_result, task):
        """Decide action for object finding task."""
        target_object = task['target_object']

        # Check if object is in current view
        for obj in perception_result['objects']:
            if obj['class'] == target_object:
                return {'type': 'navigate', 'goal': obj['location']}

        # Otherwise, continue exploring
        return self.decide_explore_action(perception_result, {'type': 'explore'})

    def decide_navigate_action(self, task):
        """Decide action for navigation task."""
        return {'type': 'navigate', 'goal': task['destination']}

    def find_unexplored_areas(self, env_map):
        """Find areas that haven't been explored."""
        # Implementation would analyze map for unknown areas
        return []

def main(args=None):
    rclpy.init(args=args)
    ai_brain = AIRobotBrain()

    # Add example tasks
    ai_brain.add_task({'type': 'explore', 'area': 'room1'})
    ai_brain.add_task({'type': 'find_object', 'target_object': 'red_cube'})

    try:
        rclpy.spin(ai_brain)
    except KeyboardInterrupt:
        pass
    finally:
        ai_brain.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

3. **Create a launch file** (`launch/ai_robot_brain.launch.py`):
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Launch Gazebo with robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'empty_world.launch.py'
                ])
            ]),
            launch_arguments={
                'world': PathJoinSubstitution([FindPackageShare('your_package'), 'worlds', 'office_world.sdf']),
                'paused': 'false',
                'use_sim_time': use_sim_time,
                'gui': 'true',
                'headless': 'false',
                'debug': 'false',
            }.items()
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[PathJoinSubstitution([FindPackageShare('your_package'), 'config', 'ai_robot.urdf'])]
        ),

        # Spawn Robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'ai_robot',
                '-file', PathJoinSubstitution([FindPackageShare('your_package'), 'config', 'ai_robot.urdf']),
                '-x', '0',
                '-y', '0',
                '-z', '0.1'
            ],
            output='screen'
        ),

        # Launch AI Robot Brain
        Node(
            package='your_package',
            executable='ai_robot_brain',
            name='ai_robot_brain',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # Launch perception pipeline
        Node(
            package='your_package',
            executable='perception_pipeline',
            name='perception_pipeline',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # Launch navigation stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'navigation_launch.py'
                ])
            ]),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )
    ])
```

### Testing the Project
1. Launch the complete system: `ros2 launch your_package ai_robot_brain.launch.py`
2. Monitor the AI brain's decision making in RViz
3. Observe perception, navigation, and task execution
4. Add tasks dynamically using ROS 2 services

## Project 2: Isaac Sim Perception Training Pipeline

### Objective
Create a pipeline that uses Isaac Sim to generate synthetic data for training perception models.

### Requirements
- Generate diverse synthetic datasets
- Include variations in lighting, textures, objects
- Automatically annotate data
- Train models using synthetic data
- Validate on real data

## Project 3: Multi-Agent AI-Robot Coordination

### Objective
Extend the AI-Robot brain to coordinate multiple robots for complex tasks.

### Requirements
- Implement multi-robot communication
- Coordinate task allocation
- Handle inter-robot collision avoidance
- Demonstrate collaborative behaviors
- Simulate in Isaac Sim environment

## Summary

These projects helped you practice:
- Creating complete AI-Robot brain systems
- Integrating perception, navigation, and decision-making
- Working with Isaac Sim for advanced robotics
- Implementing complex autonomous behaviors
- Building multi-component robotic systems

Complete these projects to solidify your understanding of AI-Robot brains before moving to Module 4.
---
sidebar_position: 4
---

# Navigation and Path Planning

Navigation and path planning form the foundation of mobile robot autonomy. In this section, we'll explore how to implement these capabilities using Isaac Sim and Isaac ROS components.

## Introduction to Robot Navigation

Navigation involves:
- **Localization**: Determining the robot's position in the environment
- **Mapping**: Creating a representation of the environment
- **Path Planning**: Finding optimal routes from start to goal
- **Motion Control**: Executing planned paths while avoiding obstacles

## Isaac ROS Navigation Components

Isaac ROS provides optimized navigation components:
- Isaac ROS Nav2 Bridge
- Hardware-accelerated path planners
- Optimized controllers
- Perception-integrated navigation

### Nav2 Integration with Isaac Sim

Example launch file for Nav2 in Isaac Sim:

```python
# launch/isaac_nav2.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration('params_file')
    default_nav2_params = PathJoinSubstitution(
        [FindPackageShare('isaac_nav2_examples'), 'params', 'nav2_params.yaml']
    )

    # Map file
    map_yaml_file = LaunchConfiguration('map')
    default_map = PathJoinSubstitution(
        [FindPackageShare('isaac_nav2_examples'), 'maps', 'simple_map.yaml']
    )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Isaac Sim) clock if true'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=default_nav2_params,
            description='Full path to the ROS2 parameters file to use for all launched nodes'
        ),

        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to map file to load'
        ),

        # Isaac Sim bridge for navigation
        Node(
            package='isaac_ros_bridges',
            executable='isaac_ros_nav_bridge',
            name='isaac_nav_bridge',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('/initialpose', 'initialpose'),
                ('/goal_pose', 'goal_pose'),
                ('/cmd_vel', 'cmd_vel'),
                ('/odom', 'odom'),
                ('/scan', 'scan'),
                ('/map', 'map'),
                ('/map_updates', 'map_updates'),
            ]
        ),

        # Navigation stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'navigation_launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file
            }.items(),
        ),

        # AMCL for localization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'localization_launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': map_yaml_file,
                'params_file': params_file
            }.items(),
        )
    ])
```

## Path Planning Algorithms

### A* Path Planning Implementation

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import heapq

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')

        # Subscriptions
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10
        )

        # Publishers
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        self.vis_pub = self.create_publisher(MarkerArray, '/path_visualization', 10)

        # Map data
        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.0
        self.map_origin = None

        # Goal
        self.goal = None

        # Path planning parameters
        self.inflation_radius = 0.5  # meters

    def map_callback(self, msg):
        """Process occupancy grid map."""
        self.map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin

    def goal_callback(self, msg):
        """Process new goal pose."""
        if self.map_data is not None:
            # Convert goal coordinates to map indices
            goal_x = msg.pose.position.x
            goal_y = msg.pose.position.y

            # Convert to map coordinates
            map_x = int((goal_x - self.map_origin.position.x) / self.map_resolution)
            map_y = int((goal_y - self.map_origin.position.y) / self.map_resolution)

            # Check bounds
            if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
                self.goal = (map_x, map_y)
                self.get_logger().info(f'New goal set: ({map_x}, {map_y})')

                # Plan path from current position
                current_pos = self.get_current_position()
                if current_pos:
                    path = self.a_star_plan(current_pos, self.goal)
                    if path:
                        self.publish_path(path)
                    else:
                        self.get_logger().warn('No path found!')

    def get_current_position(self):
        """Get current robot position from odometry (simplified)."""
        # In practice, get from tf or odometry
        # For this example, return a fixed start position
        return (10, 10)  # Example start position

    def a_star_plan(self, start, goal):
        """A* path planning algorithm."""
        if not self.is_valid(start[0], start[1]) or not self.is_valid(goal[0], goal[1]):
            return None

        # Directions: up, down, left, right, and diagonals
        directions = [
            (-1, 0), (1, 0), (0, -1), (0, 1),  # cardinal
            (-1, -1), (-1, 1), (1, -1), (1, 1)  # diagonal
        ]

        # Heuristic function (Euclidean distance)
        def heuristic(a, b):
            return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

        # Priority queue: (f_score, g_score, position)
        open_set = [(heuristic(start, goal), 0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}

        visited = set()

        while open_set:
            current_f, current_g, current = heapq.heappop(open_set)

            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path

            visited.add(current)

            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)

                # Check if neighbor is valid
                if not self.is_valid(neighbor[0], neighbor[1]):
                    continue

                if neighbor in visited:
                    continue

                # Calculate tentative g_score
                if abs(dx) + abs(dy) == 2:  # Diagonal move
                    tentative_g = current_g + 1.414  # sqrt(2)
                else:  # Cardinal move
                    tentative_g = current_g + 1.0

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + heuristic(neighbor, goal)

                    heapq.heappush(open_set, (f_score[neighbor], g_score[neighbor], neighbor))

        return None  # No path found

    def is_valid(self, x, y):
        """Check if position is valid (not occupied or outside map)."""
        if x < 0 or x >= self.map_width or y < 0 or y >= self.map_height:
            return False

        # Check if cell is occupied (value > 50 in occupancy grid)
        if self.map_data[y, x] > 50:  # Occupied
            return False

        return True

    def publish_path(self, path):
        """Publish the planned path."""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'  # Assuming map frame

        # Convert grid coordinates to world coordinates
        for x, y in path:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'

            # Convert grid to world coordinates
            pose.pose.position.x = self.map_origin.position.x + x * self.map_resolution
            pose.pose.position.y = self.map_origin.position.y + y * self.map_resolution
            pose.pose.position.z = self.map_origin.position.z

            # Set orientation to face the next point (simplified)
            if path.index((x, y)) < len(path) - 1:
                next_x, next_y = path[path.index((x, y)) + 1]
                angle = np.arctan2(next_y - y, next_x - x)

                from tf_transformations import quaternion_from_euler
                quat = quaternion_from_euler(0, 0, angle)
                pose.pose.orientation.x = quat[0]
                pose.pose.orientation.y = quat[1]
                pose.pose.orientation.z = quat[2]
                pose.pose.orientation.w = quat[3]

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Published path with {len(path)} waypoints')

def main(args=None):
    rclpy.init(args=args)
    planner = AStarPlanner()

    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Dynamic Window Approach (DWA) Local Planner

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker
import numpy as np

class DWALocalPlanner(Node):
    def __init__(self):
        super().__init__('dwa_local_planner')

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10
        )
        self.global_path_sub = self.create_subscription(
            Path, '/plan', self.global_path_callback, 10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.local_plan_pub = self.create_publisher(Path, '/local_plan', 10)
        self.viz_pub = self.create_publisher(Marker, '/dwa_trajectory', 10)

        # Robot state
        self.current_pose = None
        self.current_twist = None
        self.laser_data = None
        self.global_path = None

        # DWA parameters
        self.max_speed = 0.5  # m/s
        self.min_speed = 0.0  # m/s
        self.max_yawrate = 1.0  # rad/s
        self.max_dyawrate = 10.0  # rad/s/s
        self.v_reso = 0.05  # m/s
        self.yawrate_reso = 0.1  # rad/s
        self.dt = 0.1  # s
        self.predict_time = 1.5  # s
        self.to_goal_cost_gain = 1.0
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0
        self.robot_radius = 0.3  # m

        # Control timer
        self.control_timer = self.create_timer(0.1, self.control_callback)

    def odom_callback(self, msg):
        """Update robot state from odometry."""
        self.current_pose = msg.pose.pose
        self.current_twist = msg.twist.twist

    def laser_callback(self, msg):
        """Update laser scan data."""
        self.laser_data = msg

    def global_path_callback(self, msg):
        """Update global path."""
        self.global_path = msg.poses

    def control_callback(self):
        """Main control loop."""
        if self.current_pose is None or self.global_path is None:
            return

        # Get current goal from global path
        goal = self.get_next_waypoint()
        if goal is None:
            # Stop if no more waypoints
            cmd_vel = Twist()
            self.cmd_vel_pub.publish(cmd_vel)
            return

        # Run DWA
        cmd_vel = self.dwa_control(goal)
        self.cmd_vel_pub.publish(cmd_vel)

        # Visualize local trajectory
        self.visualize_trajectory(cmd_vel)

    def get_next_waypoint(self):
        """Get the next waypoint from the global path."""
        if not self.global_path:
            return None

        # Find closest point on path
        current_pos = np.array([self.current_pose.position.x, self.current_pose.position.y])

        closest_dist = float('inf')
        closest_idx = 0

        for i, pose_stamped in enumerate(self.global_path):
            pos = np.array([pose_stamped.pose.position.x, pose_stamped.pose.position.y])
            dist = np.linalg.norm(current_pos - pos)
            if dist < closest_dist:
                closest_dist = dist
                closest_idx = i

        # Return a point ahead in the path
        lookahead_idx = min(closest_idx + 5, len(self.global_path) - 1)
        goal_pos = self.global_path[lookahead_idx].pose.position
        return np.array([goal_pos.x, goal_pos.y])

    def dwa_control(self, goal):
        """Dynamic Window Approach for local planning."""
        # Current state
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        theta = self.get_yaw_from_quaternion(self.current_pose.orientation)
        v = self.current_twist.linear.x
        omega = self.current_twist.angular.z

        # Calculate dynamic window
        vs = [self.min_speed, self.max_speed,
              -self.max_yawrate, self.max_yawrate]
        vd = [v - self.dt * 0.5, v + self.dt * 0.5,
              omega - self.dt * self.max_dyawrate, omega + self.dt * self.max_dyawrate]

        # Limit window to feasible velocities
        dw = [max(vs[0], vd[0]), min(vs[1], vd[1]),
              max(vs[2], vd[2]), min(vs[3], vd[3])]

        # Evaluate trajectories
        best_traj = None
        best_score = float('-inf')

        v_samples = np.arange(dw[0], dw[1], self.v_reso)
        omega_samples = np.arange(dw[2], dw[3], self.yawrate_reso)

        for v_sample in v_samples:
            for omega_sample in omega_samples:
                traj = self.predict_trajectory(x, y, theta, v_sample, omega_sample)

                # Calculate costs
                to_goal_cost = self.calc_to_goal_cost(traj, goal)
                speed_cost = self.calc_speed_cost(traj)
                obs_cost = self.calc_obstacle_cost(traj)

                # Combined score (lower is better, so negate for maximization)
                score = (self.to_goal_cost_gain * to_goal_cost +
                         self.speed_cost_gain * speed_cost +
                         self.obstacle_cost_gain * obs_cost)

                if score > best_score:
                    best_score = score
                    best_traj = (v_sample, omega_sample)

        # Create command
        cmd_vel = Twist()
        if best_traj:
            cmd_vel.linear.x = best_traj[0]
            cmd_vel.angular.z = best_traj[1]
        else:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0

        return cmd_vel

    def predict_trajectory(self, x, y, theta, v, omega):
        """Predict trajectory for given velocities."""
        traj = []
        time = 0
        while time <= self.predict_time:
            x += v * np.cos(theta) * self.dt
            y += v * np.sin(theta) * self.dt
            theta += omega * self.dt
            traj.append([x, y])
            time += self.dt
        return np.array(traj)

    def calc_to_goal_cost(self, traj, goal):
        """Calculate cost to goal."""
        if len(traj) == 0:
            return float('inf')

        dx = goal[0] - traj[-1][0]
        dy = goal[1] - traj[-1][1]
        error = np.sqrt(dx**2 + dy**2)
        return error

    def calc_speed_cost(self, traj):
        """Calculate cost based on speed."""
        if len(traj) == 0:
            return float('inf')

        v = np.sqrt((traj[-1][0] - traj[0][0])**2 + (traj[-1][1] - traj[0][1])**2) / self.predict_time
        return abs(self.max_speed - v)

    def calc_obstacle_cost(self, traj):
        """Calculate cost based on obstacle proximity."""
        if self.laser_data is None or len(traj) == 0:
            return float('inf')

        min_dist = float('inf')
        for point in traj:
            # Convert trajectory point to laser frame (simplified)
            # In practice, use tf to transform coordinates
            for i, range_val in enumerate(self.laser_data.ranges):
                if not np.isfinite(range_val):
                    continue

                angle = self.laser_data.angle_min + i * self.laser_data.angle_increment
                obs_x = point[0] + range_val * np.cos(angle)
                obs_y = point[1] + range_val * np.sin(angle)

                dist = np.sqrt((obs_x - point[0])**2 + (obs_y - point[1])**2)
                min_dist = min(min_dist, dist)

        if min_dist <= self.robot_radius:
            return float('inf')
        else:
            return 1.0 / min_dist

    def get_yaw_from_quaternion(self, quaternion):
        """Extract yaw from quaternion."""
        import math
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def visualize_trajectory(self, cmd_vel):
        """Visualize the planned trajectory."""
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'dwa_trajectory'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Set start point (current position)
        marker.points = []
        start_point = Point()
        start_point.x = self.current_pose.position.x
        start_point.y = self.current_pose.position.y
        start_point.z = 0.0
        marker.points.append(start_point)

        # Set end point (predicted position)
        end_point = Point()
        end_point.x = self.current_pose.position.x + cmd_vel.linear.x * 0.5
        end_point.y = self.current_pose.position.y + cmd_vel.linear.y * 0.5
        end_point.z = 0.0
        marker.points.append(end_point)

        marker.scale.x = 0.1  # shaft diameter
        marker.scale.y = 0.2  # head diameter
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        self.viz_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    planner = DWALocalPlanner()

    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Isaac Sim Navigation Pipeline

Creating an integrated navigation system:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan, Image
from visualization_msgs.msg import MarkerArray
import numpy as np

class IsaacNavigationSystem(Node):
    def __init__(self):
        super().__init__('isaac_navigation_system')

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10
        )
        self.camera_sub = self.create_subscription(
            Image, '/camera/rgb', self.camera_callback, 10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.path_pub = self.create_publisher(Path, '/plan', 10)

        # Navigation state
        self.current_pose = None
        self.current_velocity = None
        self.laser_data = None
        self.camera_data = None

        # Navigation goals queue
        self.goals = []
        self.current_goal_index = 0

        # Navigation state machine
        self.state = 'WAITING'  # WAITING, PLANNING, EXECUTING, RECOVERY

        # Timers
        self.nav_timer = self.create_timer(0.1, self.navigation_loop)

    def odom_callback(self, msg):
        """Update current pose and velocity."""
        self.current_pose = msg.pose.pose
        self.current_velocity = msg.twist.twist

    def laser_callback(self, msg):
        """Update laser scan data."""
        self.laser_data = msg

    def camera_callback(self, msg):
        """Update camera data for visual navigation."""
        self.camera_data = msg

    def add_goal(self, x, y, theta=0.0):
        """Add a navigation goal to the queue."""
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0

        # Convert theta to quaternion
        from tf_transformations import quaternion_from_euler
        quat = quaternion_from_euler(0, 0, theta)
        goal.pose.orientation.x = quat[0]
        goal.pose.orientation.y = quat[1]
        goal.pose.orientation.z = quat[2]
        goal.pose.orientation.w = quat[3]

        self.goals.append(goal)
        self.get_logger().info(f'Added goal: ({x}, {y}, {theta})')

    def navigation_loop(self):
        """Main navigation state machine."""
        if self.current_pose is None:
            return

        if self.state == 'WAITING':
            if self.goals and self.current_goal_index < len(self.goals):
                self.state = 'PLANNING'
                self.get_logger().info('Switching to PLANNING state')

        elif self.state == 'PLANNING':
            # Plan path to current goal
            goal = self.goals[self.current_goal_index]
            success = self.plan_path_to_goal(goal)

            if success:
                self.state = 'EXECUTING'
                self.get_logger().info('Switching to EXECUTING state')
            else:
                self.state = 'RECOVERY'
                self.get_logger().warn('Planning failed, switching to RECOVERY')

        elif self.state == 'EXECUTING':
            # Execute planned path
            goal_reached = self.execute_path()

            if goal_reached:
                self.current_goal_index += 1
                if self.current_goal_index >= len(self.goals):
                    self.get_logger().info('All goals completed!')
                    self.state = 'WAITING'
                    self.stop_robot()
                else:
                    self.state = 'PLANNING'
            else:
                # Check for obstacles or other issues
                if self.detect_obstacles():
                    self.state = 'RECOVERY'
                    self.get_logger().info('Obstacle detected, switching to RECOVERY')

        elif self.state == 'RECOVERY':
            # Perform recovery behaviors
            recovered = self.perform_recovery()

            if recovered:
                self.state = 'PLANNING'
                self.get_logger().info('Recovery successful, returning to PLANNING')
            else:
                # Still in recovery, continue
                pass

    def plan_path_to_goal(self, goal):
        """Plan path to the given goal."""
        # In Isaac ROS, this would call the global planner
        # For now, we'll publish a simple path command
        self.get_logger().info(f'Planning to goal: ({goal.pose.position.x}, {goal.pose.position.y})')

        # Publish goal for Nav2
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose = goal.pose

        self.goal_pub.publish(goal_msg)
        return True

    def execute_path(self):
        """Execute the planned path."""
        # In Isaac ROS, this would use the local planner and controller
        # For now, we'll implement a simple proportional controller

        if not self.goals or self.current_goal_index >= len(self.goals):
            return True

        goal = self.goals[self.current_goal_index].pose
        current_pos = self.current_pose.position

        # Calculate distance to goal
        dist_to_goal = np.sqrt(
            (goal.position.x - current_pos.x)**2 +
            (goal.position.y - current_pos.y)**2
        )

        # Check if goal is reached
        if dist_to_goal < 0.2:  # 20 cm tolerance
            self.get_logger().info('Goal reached!')
            return True

        # Simple proportional controller
        cmd_vel = Twist()
        cmd_vel.linear.x = min(0.3, max(0.0, dist_to_goal * 0.5))  # Proportional to distance

        # Calculate heading to goal
        angle_to_goal = np.arctan2(
            goal.position.y - current_pos.y,
            goal.position.x - current_pos.x
        )

        # Current robot orientation
        from tf_transformations import euler_from_quaternion
        (_, _, current_yaw) = euler_from_quaternion([
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        ])

        # Angle difference
        angle_diff = angle_to_goal - current_yaw
        while angle_diff > np.pi:
            angle_diff -= 2 * np.pi
        while angle_diff < -np.pi:
            angle_diff += 2 * np.pi

        cmd_vel.angular.z = max(-0.5, min(0.5, angle_diff * 2.0))  # Proportional to angle error

        self.cmd_vel_pub.publish(cmd_vel)
        return False

    def detect_obstacles(self):
        """Detect obstacles using laser data."""
        if self.laser_data is None:
            return False

        # Check for obstacles in front of robot (simplified)
        front_range = len(self.laser_data.ranges) // 2
        front_left = front_range - 10
        front_right = front_range + 10

        min_dist = float('inf')
        for i in range(front_left, front_right + 1):
            if i < len(self.laser_data.ranges):
                dist = self.laser_data.ranges[i]
                if np.isfinite(dist):
                    min_dist = min(min_dist, dist)

        return min_dist < 0.5  # Obstacle within 50 cm

    def perform_recovery(self):
        """Perform recovery behaviors."""
        # Simple recovery: backup and turn
        cmd_vel = Twist()
        cmd_vel.linear.x = -0.2  # Back up
        cmd_vel.angular.z = 0.5  # Turn
        self.cmd_vel_pub.publish(cmd_vel)

        # Recovery after 2 seconds
        import time
        time.sleep(2.0)
        self.stop_robot()

        return True  # Recovery successful

    def stop_robot(self):
        """Stop the robot."""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    nav_system = IsaacNavigationSystem()

    # Add some example goals
    nav_system.add_goal(5.0, 0.0)
    nav_system.add_goal(5.0, 3.0)
    nav_system.add_goal(0.0, 3.0)
    nav_system.add_goal(0.0, 0.0)  # Return to start

    try:
        rclpy.spin(nav_system)
    except KeyboardInterrupt:
        pass
    finally:
        nav_system.stop_robot()
        nav_system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices for Navigation

1. **Safe Speed Control**: Adjust speeds based on sensor reliability
2. **Robust Localization**: Use multiple sensors for reliable positioning
3. **Dynamic Replanning**: Adapt plans when obstacles are detected
4. **Recovery Behaviors**: Implement fallback strategies for failures
5. **Simulation Validation**: Test extensively in simulation before real deployment

## Exercise

Create a complete navigation system that:
1. Implements both global and local planning
2. Uses Isaac Sim sensors for obstacle detection
3. Handles dynamic obstacles
4. Includes recovery behaviors
5. Integrates with Isaac ROS navigation stack

## Summary

In this section, you learned:
- How to implement path planning algorithms like A*
- How to create local planners using DWA
- How to build complete navigation systems
- How to integrate with Isaac ROS navigation components
- Best practices for robust navigation

In the next section, we'll explore projects that combine all navigation concepts.
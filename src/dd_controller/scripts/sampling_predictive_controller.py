#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped, Quaternion, Pose
from nav_msgs.msg import Path, OccupancyGrid
from rclpy.qos import QoSProfile
import numpy as np
import math
from tf_transformations import euler_from_quaternion

class LocalPlannerControllerNode(Node):
    def __init__(self):
        super().__init__('local_planner_controller')
        qos = QoSProfile(depth=10)
        
        # Declare parameters
        self.declare_parameter('horizon', 2.5)         # seconds to simulate ahead
        self.declare_parameter('dt', 0.1)              # simulation time step
        self.declare_parameter('num_samples_v', 7)     # candidate linear speed samples (forward and reverse)
        self.declare_parameter('num_samples_w', 7)     # candidate angular speed samples
        self.declare_parameter('v_max', 0.5)           # max forward linear speed [m/s]
        self.declare_parameter('w_max', 1.0)           # max angular speed [rad/s]
        self.declare_parameter('obstacle_weight', 15.0)    # penalty weight for obstacle proximity
        self.declare_parameter('path_weight', 0.05)         # weight for global path deviation (minimal bias)
        self.declare_parameter('forward_reward', 0.1)      # reward for forward progress
        self.declare_parameter('obstacle_threshold', 30)   # cell cost threshold to trigger penalty
        self.declare_parameter('penalty_factor', 300)      # penalty factor for imminent collisions
        self.declare_parameter('emergency_ttc', 0.5)       # if collision predicted within 0.5 s, reject trajectory
        self.declare_parameter('clearance_penalty_weight', 2.0)  # penalize trajectories with low clearance
        self.declare_parameter('emergency_stop_distance', 0.2)   # reactive emergency stop threshold
        self.declare_parameter('path_update_threshold', 60)      # update waypoint only if cost > 60
        self.declare_parameter('path_update_delta', 10)         # update if improvement is at least 10
        
        # Retrieve parameters
        self.horizon = self.get_parameter('horizon').value
        self.dt = self.get_parameter('dt').value
        self.num_samples_v = self.get_parameter('num_samples_v').value
        self.num_samples_w = self.get_parameter('num_samples_w').value
        self.v_max = self.get_parameter('v_max').value
        self.w_max = self.get_parameter('w_max').value
        self.obstacle_weight = self.get_parameter('obstacle_weight').value
        self.path_weight = self.get_parameter('path_weight').value
        self.forward_reward = self.get_parameter('forward_reward').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.penalty_factor = self.get_parameter('penalty_factor').value
        self.emergency_ttc = self.get_parameter('emergency_ttc').value
        self.clearance_penalty_weight = self.get_parameter('clearance_penalty_weight').value
        self.emergency_stop_distance = self.get_parameter('emergency_stop_distance').value
        self.path_update_threshold = self.get_parameter('path_update_threshold').value
        self.path_update_delta = self.get_parameter('path_update_delta').value
        
        # Subscribers for AMCL pose, global path, and local costmap.
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.pose_callback, qos)
        self.global_path_sub = self.create_subscription(Path, "/global_path", self.global_path_callback, qos)
        self.costmap_sub = self.create_subscription(OccupancyGrid, "/local_costmap", self.costmap_callback, qos)
        
        # Publisher for velocity commands.
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", qos)
        
        # Publisher for local path visualization.
        self.local_path_pub = self.create_publisher(Path, "/local_path", qos)
        
        # Control loop timer at 10 Hz.
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Internal state
        self.current_pose = None
        self.global_path = None
        self.local_costmap = None
        
        self.get_logger().info("Local Planner & Controller Node finalized without forced turn.")

    # -------------------- Callbacks --------------------
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg.pose.pose

    def global_path_callback(self, msg: Path):
        self.global_path = msg
        self.get_logger().debug(f"Global path received with {len(msg.poses)} poses.")

    def costmap_callback(self, msg: OccupancyGrid):
        self.local_costmap = msg

    # -------------------- Main Control Loop --------------------
    def control_loop(self):
        if self.current_pose is None or self.global_path is None or self.local_costmap is None:
            return

        # Update the global path using local costmap information.
        updated_path = self.update_global_path_with_local_costmap()
        if updated_path is not None:
            self.global_path = updated_path
            self.get_logger().info("Global path updated based on local costmap.")
        
        # Emergency stop check.
        if self.is_emergency_stop_needed():
            self.get_logger().warn("Emergency stop triggered: obstacle too near!")
            self.cmd_vel_pub.publish(Twist())
            return

        # Check if goal is reached.
        goal_pose = self.global_path.poses[-1].pose
        dist_to_goal = math.hypot(self.current_pose.position.x - goal_pose.position.x,
                                  self.current_pose.position.y - goal_pose.position.y)
        if dist_to_goal < 0.3:
            self.get_logger().info("Goal reached; stopping robot.")
            self.cmd_vel_pub.publish(Twist())
            return

        # Normal forward candidate sampling.
        best_cost = float('inf')
        best_control = None
        best_trajectory = None
        v_candidates = np.linspace(0.0, self.v_max, self.num_samples_v)
        w_candidates = np.linspace(-self.w_max, self.w_max, self.num_samples_w)
        for v in v_candidates:
            for w in w_candidates:
                cost, trajectory = self.evaluate_trajectory(v, w)
                if cost < best_cost:
                    best_cost = cost
                    best_control = (v, w)
                    best_trajectory = trajectory
        
        if best_control is None:
            self.get_logger().warn("No valid forward trajectory found; attempting reverse mode.")
            best_control, best_trajectory = self.attempt_reverse()

        if best_control is None:
            self.get_logger().warn("No valid trajectory found even in reverse; attempting to rotate.")
            best_control, best_trajectory = self.attempt_rotation()

        if best_control is not None:
            twist = Twist()
            twist.linear.x = best_control[0]
            twist.angular.z = best_control[1]
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(f"Chosen control: v={best_control[0]:.2f}, w={best_control[1]:.2f}")
            
            # Publish the local path for visualization.
            if best_trajectory:
                self.publish_local_path(best_trajectory)
        else:
            self.get_logger().warn("No valid trajectory found; stopping robot.")
            self.cmd_vel_pub.publish(Twist())

    # -------------------- Reverse Mode --------------------
    def attempt_reverse(self):
        best_cost = float('inf')
        best_control = None
        best_trajectory = None
        v_candidates = np.linspace(-0.3, -0.1, self.num_samples_v)
        w_candidates = np.linspace(-self.w_max, self.w_max, self.num_samples_w)
        for v in v_candidates:
            for w in w_candidates:
                cost, trajectory = self.evaluate_trajectory(v, w)
                if cost < best_cost:
                    best_cost = cost
                    best_control = (v, w)
                    best_trajectory = trajectory
        if best_control:
            self.get_logger().info(f"Reverse mode selected: v={best_control[0]:.2f}, w={best_control[1]:.2f}")
        return best_control, best_trajectory

    # -------------------- Rotation Mode --------------------
    def attempt_rotation(self):
        """Attempt to rotate the robot to find a safe direction."""
        # Try rotating to the right first.
        right_cost, right_trajectory = self.evaluate_trajectory(0.0, -self.w_max)  # Negative angular velocity for right turn
        left_cost, left_trajectory = self.evaluate_trajectory(0.0, self.w_max)    # Positive angular velocity for left turn

        if right_cost < left_cost and right_cost != float('inf'):
            self.get_logger().info("Rotating to the right.")
            return (0.0, -self.w_max), right_trajectory  # Rotate right
        elif left_cost != float('inf'):
            self.get_logger().info("Rotating to the left.")
            return (0.0, self.w_max), left_trajectory   # Rotate left
        else:
            self.get_logger().warn("No safe rotation found.")
            return None, None

        # -------------------- Global Path Update --------------------
    def update_global_path_with_local_costmap(self):
        """Update each global path waypoint only if its cost exceeds a threshold and a nearby free cell exists."""
        if self.global_path is None or self.local_costmap is None:
            return self.global_path
        updated_poses = []
        for pose_stamped in self.global_path.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            cost = self.get_cost_at_position(x, y)
            if cost > self.path_update_threshold:
                new_x, new_y = self.find_nearest_free_cell(x, y)
                candidate_cost = self.get_cost_at_position(new_x, new_y)
                if cost - candidate_cost > self.path_update_delta:
                    new_pose = PoseStamped()
                    new_pose.header = pose_stamped.header
                    new_pose.pose.position.x = new_x
                    new_pose.pose.position.y = new_y
                    new_pose.pose.position.z = 0.0
                    new_pose.pose.orientation = pose_stamped.pose.orientation
                    updated_poses.append(new_pose)
                else:
                    updated_poses.append(pose_stamped)
            else:
                updated_poses.append(pose_stamped)
        new_path = Path()
        new_path.header = self.global_path.header
        new_path.poses = updated_poses
        return new_path

    def find_nearest_free_cell(self, x, y):
        res = self.local_costmap.info.resolution
        origin_x = self.local_costmap.info.origin.position.x
        origin_y = self.local_costmap.info.origin.position.y
        width = self.local_costmap.info.width
        height = self.local_costmap.info.height
        gx = int((x - origin_x) / res)
        gy = int((y - origin_y) / res)
        best_gx, best_gy = gx, gy
        best_cost = self.get_cost_at_position(x, y)
        search_radius = 3  # cells
        for dx in range(-search_radius, search_radius + 1):
            for dy in range(-search_radius, search_radius + 1):
                nx = gx + dx
                ny = gy + dy
                if nx < 0 or ny < 0 or nx >= width or ny >= height:
                    continue
                wx, wy = self.grid_to_world(nx, ny)
                cell_cost = self.get_cost_at_position(wx, wy)
                if cell_cost < best_cost:
                    best_cost = cell_cost
                    best_gx, best_gy = nx, ny
        return self.grid_to_world(best_gx, best_gy)

    def grid_to_world(self, gx, gy):
        res = self.local_costmap.info.resolution
        origin_x = self.local_costmap.info.origin.position.x
        origin_y = self.local_costmap.info.origin.position.y
        x = gx * res + origin_x + res / 2.0
        y = gy * res + origin_y + res / 2.0
        return x, y

    # -------------------- Trajectory Evaluation --------------------
    def evaluate_trajectory(self, v, w):
        if self.current_pose is None:
            return float('inf'), None
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        yaw = self.get_yaw_from_pose(self.current_pose)
        total_cost = 0.0
        steps = int(self.horizon / self.dt)
        min_cell_cost = 100
        trajectory = []
        for step in range(steps):
            x += v * math.cos(yaw) * self.dt
            y += v * math.sin(yaw) * self.dt
            yaw += w * self.dt
            cell_cost = self.get_cost_at_position(x, y)
            min_cell_cost = min(min_cell_cost, cell_cost)
            if cell_cost >= self.obstacle_threshold:
                ttc = step * self.dt
                if ttc < self.emergency_ttc:
                    return float('inf'), None
                penalty = (self.horizon - ttc) * self.penalty_factor
                total_cost += penalty
            else:
                total_cost += self.obstacle_weight * (cell_cost / 100.0)
            path_error = self.distance_to_global_path(x, y)
            total_cost += self.path_weight * path_error
            total_cost -= self.forward_reward * v
            
            # Store the trajectory for visualization.
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.0
            quat = self.get_quaternion_from_yaw(yaw)
            pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            trajectory.append(pose)
        
        total_cost += self.clearance_penalty_weight * min_cell_cost
        return total_cost, trajectory

    # -------------------- Publish Local Path --------------------
    def publish_local_path(self, trajectory):
        """Publish the local path for visualization in Rviz2."""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"  # Assuming the map frame is used.
        
        for pose in trajectory:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = path_msg.header.stamp
            pose_stamped.header.frame_id = path_msg.header.frame_id
            pose_stamped.pose = pose
            path_msg.poses.append(pose_stamped)
        
        self.local_path_pub.publish(path_msg)

    # -------------------- Helper Functions --------------------
    def get_quaternion_from_yaw(self, yaw):
        """Convert a yaw angle to a Quaternion."""
        return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

    def get_cost_at_position(self, x, y):
        if self.local_costmap is None:
            return 0
        res = self.local_costmap.info.resolution
        origin_x = self.local_costmap.info.origin.position.x
        origin_y = self.local_costmap.info.origin.position.y
        width = self.local_costmap.info.width
        height = self.local_costmap.info.height
        gx = int((x - origin_x) / res)
        gy = int((y - origin_y) / res)
        if gx < 0 or gy < 0 or gx >= width or gy >= height:
            return 100
        index = gy * width + gx
        cost = self.local_costmap.data[index]
        if cost < 0:
            return 50
        return cost

    def distance_to_global_path(self, x, y):
        if not self.global_path or len(self.global_path.poses) == 0:
            return 0.0
        min_dist = float('inf')
        for pose_stamped in self.global_path.poses:
            dx = x - pose_stamped.pose.position.x
            dy = y - pose_stamped.pose.position.y
            d = math.hypot(dx, dy)
            if d < min_dist:
                min_dist = d
        return min_dist

    def get_heading_error_to_goal(self):
        if not self.global_path or len(self.global_path.poses) == 0:
            return 0.0
        goal_pose = self.global_path.poses[-1].pose
        gx, gy = goal_pose.position.x, goal_pose.position.y
        rx, ry = self.current_pose.position.x, self.current_pose.position.y
        ryaw = self.get_yaw_from_pose(self.current_pose)
        dx = gx - rx
        dy = gy - ry
        goal_yaw = math.atan2(dy, dx)
        return self.normalize_angle(goal_yaw - ryaw)

    def get_yaw_from_pose(self, pose):
        quat = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        (_, _, yaw) = euler_from_quaternion(quat)
        return yaw

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def is_emergency_stop_needed(self):
        if self.current_pose is None or self.local_costmap is None:
            return False
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        yaw = self.get_yaw_from_pose(self.current_pose)
        lookahead = 0.3
        check_x = robot_x + lookahead * math.cos(yaw)
        check_y = robot_y + lookahead * math.sin(yaw)
        cost = self.get_cost_at_position(check_x, check_y)
        return cost > 80

def main(args=None):
    rclpy.init(args=args)
    node = LocalPlannerControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
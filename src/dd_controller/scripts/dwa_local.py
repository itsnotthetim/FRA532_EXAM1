#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import numpy as np
import math
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import Twist, PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class DwaLocalPlannerNode(Node):
    def __init__(self):
        super().__init__('dwa_local_planner')

        # -------- Declare Parameters --------
        self.declare_parameters(
            namespace='',
            parameters=[
                ('control_rate', 100.0),
                ('time_ahead', 2.0),
                ('dt_sim', 0.2),
                ('max_speed', 1.5),
                ('min_speed', 0.0),
                ('max_yaw_rate', 1.0),
                ('max_accel', 0.9),
                ('max_yaw_accel', 1.0),
                ('robot_radius', 0.8),
                ('path_look_ahead', 0.8),
                ('heading_weight', 1.0),
                ('clearance_weight', 1.0),
                ('velocity_weight', 0.1)
            ]
        )

        # Load parameters
        self.control_rate = self.get_parameter('control_rate').value
        self.time_ahead = self.get_parameter('time_ahead').value
        self.dt_sim = self.get_parameter('dt_sim').value
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').value
        self.max_accel = self.get_parameter('max_accel').value
        self.max_yaw_accel = self.get_parameter('max_yaw_accel').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.path_look_ahead = self.get_parameter('path_look_ahead').value

        self.heading_weight = self.get_parameter('heading_weight').value
        self.clearance_weight = self.get_parameter('clearance_weight').value
        self.velocity_weight = self.get_parameter('velocity_weight').value

        self.control_period = 1.0 / self.control_rate

        # -------- ROS2 Interfaces --------
        qos_profile = QoSProfile(depth=10)
        
        qos_scan = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        # Global path subscription
        self.global_path_sub = self.create_subscription(
            Path, '/global_path', self.global_path_callback, qos_profile
        )

        # Robot’s current pose (e.g. AMCL)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, qos_profile
        )

        # LIDAR scan subscription
        self.laser_scan_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_scan_callback, qos_scan
        )

        # Velocity command publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)

        # Local path publisher (for visualization)
        self.local_path_pub = self.create_publisher(Path, '/local_path', qos_profile)

        # -------- Internal State --------
        self.global_path = []
        self.current_pose = None  # (x, y, yaw)
        self.current_speed = 0.0
        self.current_yaw_rate = 0.0
        self.laser_scan = None

        # Timer loop
        self.control_timer = self.create_timer(self.control_period, self.control_loop)

        self.get_logger().info("DWA Local Planner node started.")

    # -------------------- Subscription Callbacks --------------------
    def global_path_callback(self, msg: Path):
        """Convert global path into a list of (x, y)."""
        self.global_path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Store current robot pose as (x, y, yaw)."""
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_pose = (px, py, yaw)

    def laser_scan_callback(self, msg: LaserScan):
        """Store the latest LIDAR scan."""
        self.laser_scan = msg

    # -------------------- Main Control Loop --------------------
    def control_loop(self):
        # Check for missing data
        if not self.global_path or self.current_pose is None or self.laser_scan is None:
            self.get_logger().warn("Waiting for global path, robot pose, or LIDAR scan...")
            self.stop_robot()
            return

        # 1. Find local sub-goal
        gx, gy = self.find_local_goal()

        # 2. Run DWA
        best_v, best_omega, best_traj = self.dwa_planning(gx, gy)

        # 3. Publish velocity
        self.publish_velocity(best_v, best_omega)

        # 4. Publish chosen local path
        self.publish_local_path(best_traj)

    def stop_robot(self):
        """Stop the robot by publishing zero velocity."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    def dwa_planning(self, goal_x, goal_y):
        rx, ry, ryaw = self.current_pose

        # Dynamic window
        v_min = max(self.min_speed, self.current_speed - self.max_accel * self.control_period)
        v_max = min(self.max_speed, self.current_speed + self.max_accel * self.control_period)

        omega_min = self.current_yaw_rate - self.max_yaw_accel * self.control_period
        omega_max = self.current_yaw_rate + self.max_yaw_accel * self.control_period

        best_cost = float('inf')
        best_v = 0.0
        best_omega = 0.0
        best_traj = []

        v_samples = 5
        omega_samples = 5

        for iv in range(v_samples + 1):
            v = v_min + (v_max - v_min) * (iv / max(1, v_samples))
            for iw in range(omega_samples + 1):
                omega = omega_min + (omega_max - omega_min) * (iw / max(1, omega_samples))

                traj = self.simulate_trajectory(rx, ry, ryaw, v, omega)
                if self.check_collision_with_lidar(traj):
                    continue

                heading_cost = self.calc_heading_cost(traj[-1], (goal_x, goal_y))
                clearance_cost = self.calc_clearance_with_lidar(traj)
                velocity_cost = (self.max_speed - v)  # prefer higher velocity => smaller cost

                total_cost = (self.heading_weight * heading_cost
                              + self.clearance_weight * clearance_cost
                              + self.velocity_weight * velocity_cost)

                if total_cost < best_cost:
                    best_cost = total_cost
                    best_v = v
                    best_omega = omega
                    best_traj = traj

        return best_v, best_omega, best_traj

    def simulate_trajectory(self, x, y, yaw, v, omega):
        """Forward-simulate constant (v, omega) for time_ahead in increments of dt_sim."""
        traj = []
        t = 0.0
        while t < self.time_ahead:
            x += v * math.cos(yaw) * self.dt_sim
            y += v * math.sin(yaw) * self.dt_sim
            yaw += omega * self.dt_sim
            yaw = self.normalize_angle(yaw)
            traj.append((x, y, yaw))
            t += self.dt_sim
        return traj

    def check_collision_with_lidar(self, traj):
        """Check if a trajectory collides with any LIDAR points."""
        if self.laser_scan is None:
            return False

        angle_min = self.laser_scan.angle_min
        angle_increment = self.laser_scan.angle_increment
        ranges = self.laser_scan.ranges

        for (x, y, _) in traj:
            for i, range in enumerate(ranges):
                if range < self.laser_scan.range_min or range > self.laser_scan.range_max:
                    continue

                # Calculate the position of the LIDAR point
                angle = angle_min + i * angle_increment
                lx = x + range * np.cos(angle)
                ly = y + range * np.sin(angle)

                # Check if the trajectory point is too close to the LIDAR point
                dist = math.hypot(x - lx, y - ly)
                if dist < self.robot_radius:
                    return True

        return False

    def calc_heading_cost(self, last_pose, goal):
        """Calculate the heading cost based on the goal direction."""
        lx, ly, lyaw = last_pose
        gx, gy = goal
        goal_angle = math.atan2(gy - ly, gx - lx)
        diff = abs(self.normalize_angle(goal_angle - lyaw))
        return diff

    def calc_clearance_with_lidar(self, traj):
        """Calculate the clearance cost based on LIDAR data."""
        if self.laser_scan is None:
            return 0.0

        angle_min = self.laser_scan.angle_min
        angle_increment = self.laser_scan.angle_increment
        ranges = self.laser_scan.ranges

        min_dist = float('inf')
        for (x, y, _) in traj:
            for i, range in enumerate(ranges):
                if range < self.laser_scan.range_min or range > self.laser_scan.range_max:
                    continue

                # Calculate the position of the LIDAR point
                angle = angle_min + i * angle_increment
                lx = x + range * np.cos(angle)
                ly = y + range * np.sin(angle)

                # Compute the distance to the LIDAR point
                dist = math.hypot(x - lx, y - ly)
                if dist < min_dist:
                    min_dist = dist

        if min_dist == float('inf'):
            return 0.0
        return 1.0 / (1e-5 + min_dist)

    def find_local_goal(self):
        """Find the local goal from the global path."""
        if not self.global_path:
            return 0.0, 0.0

        rx, ry, _ = self.current_pose
        chosen_x, chosen_y = self.global_path[-1]  # Default to the final goal

        for (px, py) in self.global_path:
            dist = math.hypot(px - rx, py - ry)
            if dist >= self.path_look_ahead:
                chosen_x, chosen_y = px, py
                break  # Use the first point that is >= path_look_ahead distance away

        return chosen_x, chosen_y

    def publish_velocity(self, v, omega):
        """Publish the velocity command."""
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = omega
        self.cmd_vel_pub.publish(cmd)

    def publish_local_path(self, traj):
        """Publish the chosen local path for visualization."""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        for (x, y, yaw) in traj:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)
            pose_stamped.pose.orientation.x = qx
            pose_stamped.pose.orientation.y = qy
            pose_stamped.pose.orientation.z = qz
            pose_stamped.pose.orientation.w = qw
            path_msg.poses.append(pose_stamped)

        self.local_path_pub.publish(path_msg)

    def normalize_angle(self, angle):
        """Normalize an angle to the range [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = DwaLocalPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
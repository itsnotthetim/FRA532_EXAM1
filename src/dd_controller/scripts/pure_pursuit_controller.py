#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import numpy as np

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path, OccupancyGrid
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class PurePursuitLocalCostmapNode(Node):
    """
    A ROS2 node:
      - Subscribes to a global path (/global_path).
      - Subscribes to a local costmap (/local_costmap).
      - Subscribes to the robot pose (/amcl_pose).
      - Performs pure pursuit with final approach logic so the robot doesn't overshoot the final target.
      - Publishes /cmd_vel (Twist).
    """

    def __init__(self):
        super().__init__('pure_pursuit_local_controller')

        # ---------------- Parameters ----------------
        self.declare_parameter('lookahead_distance', 1.0)       # [m]
        self.declare_parameter('max_linear_speed', 0.5)         # [m/s]
        self.declare_parameter('max_angular_speed', 1.0)        # [rad/s]
        self.declare_parameter('goal_tolerance', 0.2)           # [m] stop if within this distance of final waypoint
        self.declare_parameter('use_dynamic_lookahead', True)   # if True, reduce lookahead if near final goal
        self.declare_parameter('path_topic', '/global_path')
        self.declare_parameter('pose_topic', '/amcl_pose')
        self.declare_parameter('costmap_topic', '/local_costmap')
        self.declare_parameter('control_frequency', 100.0)        # [Hz]

        # Retrieve param values
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.use_dynamic_lookahead = self.get_parameter('use_dynamic_lookahead').value
        self.path_topic = self.get_parameter('path_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.costmap_topic = self.get_parameter('costmap_topic').value
        self.control_frequency = self.get_parameter('control_frequency').value

        # Storage
        self.current_pose = None   # (x, y, yaw)
        self.current_path = []     # list of (x, y) points
        self.local_costmap = None  # 2D array from OccupancyGrid
        self.local_costmap_info = None

        # Publisher for velocity commands
        qos_profile = QoSProfile(depth=10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)

        # Subscriptions
        self.path_sub = self.create_subscription(Path, self.path_topic, self.path_callback, qos_profile)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, self.pose_topic, self.pose_callback, qos_profile)
        self.costmap_sub = self.create_subscription(OccupancyGrid, self.costmap_topic, self.costmap_callback, qos_profile)

        # Timer for control loop
        self.control_timer = self.create_timer(1.0 / self.control_frequency, self.control_loop)

        self.get_logger().info("PurePursuitLocalCostmapNode started with final approach logic.")

    # -------------- Subscriptions --------------

    def path_callback(self, msg: Path):
        """Convert the incoming Path into a list of (x,y)."""
        self.current_path = []
        for pose_stamped in msg.poses:
            px = pose_stamped.pose.position.x
            py = pose_stamped.pose.position.y
            self.current_path.append((px, py))

        self.get_logger().info(f"Received path with {len(self.current_path)} waypoints.")

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Extract (x,y,yaw) from the robot's pose (e.g., AMCL)."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        yaw = self.quaternion_to_yaw(qx, qy, qz, qw)
        self.current_pose = (x, y, yaw)

    def costmap_callback(self, msg: OccupancyGrid):
        """Store the local costmap as a 2D NumPy array for obstacle checks."""
        w = msg.info.width
        h = msg.info.height
        array_data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        self.local_costmap = array_data
        self.local_costmap_info = msg.info

    # -------------- Control Loop --------------

    def control_loop(self):
        """Main pure pursuit logic with local costmap checks and final approach."""
        if (self.current_pose is None or
            not self.current_path or
            self.local_costmap is None):
            return  # Not ready

        rx, ry, ryaw = self.current_pose
        # 1) Check distance to final waypoint
        final_x, final_y = self.current_path[-1]
        dist_to_goal = math.hypot(final_x - rx, final_y - ry)
        # If within goal tolerance => stop
        if dist_to_goal < self.goal_tolerance:
            self.get_logger().info("Within goal tolerance. Stopping.")
            self.publish_cmd_vel(0.0, 0.0)
            return

        # 2) Possibly reduce lookahead if near the goal
        lookahead = self.lookahead_distance
        if self.use_dynamic_lookahead and dist_to_goal < 2.0 * self.lookahead_distance:
            # shrink lookahead
            lookahead = dist_to_goal * 0.8  # for example
            if lookahead < 0.1:
                lookahead = 0.1  # minimum

        # 3) Find the lookahead point on the path
        lx, ly = self.find_lookahead_point(lookahead)
        # if we got the same final approach => just skip
        if (lx, ly) == (rx, ry):
            self.get_logger().info("No lookahead found or path ended. Stopping.")
            self.publish_cmd_vel(0.0, 0.0)
            return

        # 4) Check if that lookahead is blocked in the local costmap
        if self.is_blocked(lx, ly):
            self.get_logger().warn("Lookahead point is blocked in local costmap. Stopping to avoid collision.")
            self.publish_cmd_vel(0.0, 0.0)
            return

        # 5) Compute pure pursuit
        dx = lx - rx
        dy = ly - ry
        Ld = math.sqrt(dx*dx + dy*dy)
        if Ld < 1e-3:
            self.publish_cmd_vel(0.0, 0.0)
            return

        heading_to_look = math.atan2(dy, dx)
        alpha = self.angle_diff(heading_to_look, ryaw)
        curvature = 2.0 * math.sin(alpha) / Ld

        # 6) Compute velocity
        v = self.max_linear_speed
        w = curvature * v
        if abs(w) > self.max_angular_speed:
            w = math.copysign(self.max_angular_speed, w)

        # 7) Publish
        self.publish_cmd_vel(v, w)

    # -------------- Helper Methods --------------

    def find_lookahead_point(self, lookahead_distance):
        """
        Return a point (lx, ly) on current_path about 'lookahead_distance' away from the robot.
        If no point is that far, return the last point.
        """
        if len(self.current_path) < 2:
            # Not enough points => return current pose
            if self.current_pose:
                return self.current_pose[0], self.current_pose[1]
            else:
                return (0.0, 0.0)

        rx, ry, _ = self.current_pose
        for i in range(len(self.current_path)-1):
            px, py = self.current_path[i]
            dist = math.hypot(px - rx, py - ry)
            if dist >= lookahead_distance:
                return (px, py)

        # If never found a point >= lookahead_distance, use last path point
        return self.current_path[-1]

    def is_blocked(self, gx, gy):
        """Check if (gx, gy) in global coords is an occupied cell (>=100) in the local costmap."""
        if not self.local_costmap_info:
            return False  # no costmap info yet

        # transform (gx, gy) -> costmap indices
        origin_x = self.local_costmap_info.origin.position.x
        origin_y = self.local_costmap_info.origin.position.y
        res = self.local_costmap_info.resolution
        w = self.local_costmap_info.width
        h = self.local_costmap_info.height

        cx = int((gx - origin_x) / res)
        cy = int((gy - origin_y) / res)

        if (cx < 0 or cx >= w or cy < 0 or cy >= h):
            return False  # out of costmap => assume free

        cell_val = self.local_costmap[cy, cx]
        return (cell_val >= 100)

    def publish_cmd_vel(self, linear, angular):
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_vel_pub.publish(cmd)

    def quaternion_to_yaw(self, qx, qy, qz, qw):
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    def angle_diff(self, a, b):
        """Compute minimal difference between angles a, b in [-pi, pi]."""
        d = a - b
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        return d


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitLocalCostmapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()

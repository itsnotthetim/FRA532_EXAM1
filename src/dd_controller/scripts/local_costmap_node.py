#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import numpy as np
import math

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
import tf2_ros
from scipy.ndimage import binary_dilation  # For obstacle inflation
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class LocalCostmapNode(Node):
    """
    ROS2 node for a **robot-centric local costmap** (5m x 5m).
    - Uses **LiDAR data** (/scan) to detect obstacles.
    - **Inflates obstacles** to create a safety buffer.
    - Publishes a **real-time occupancy grid** on `/local_costmap`.
    - Ensures obstacles **do not rotate** with the robot.
    - Uses **TF to attach the costmap** to the robot in RViz.
    """

    def __init__(self):
        super().__init__('local_costmap_node')

        # --- Declare Parameters ---
        self.declare_parameter('local_size', 5.0)        # 5x5 meters
        self.declare_parameter('resolution', 0.05)       # meters per cell
        self.declare_parameter('inflation_radius', 0.3)  # meters
        self.declare_parameter('costmap_frame_id', 'map')  # Frame for costmap
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('pose_topic', '/amcl_pose')
        self.declare_parameter('costmap_topic', '/local_costmap')
        self.declare_parameter('publish_rate', 100.0)     # Hz

        # --- Read Parameters ---
        self.local_size = self.get_parameter('local_size').value
        self.resolution = self.get_parameter('resolution').value
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.costmap_frame_id = self.get_parameter('costmap_frame_id').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.costmap_topic = self.get_parameter('costmap_topic').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # --- Derived Parameters ---
        self.grid_size = int(self.local_size / self.resolution)  # Number of cells

        # --- Internal Storage ---
        self.local_grid = np.full((self.grid_size, self.grid_size), -1, dtype=np.int8)  # -1 = unknown
        self.robot_pose = None  # (x, y, yaw)

        # --- TF Broadcaster ---
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # --- QoS Profiles ---
        qos_profile = QoSProfile(depth=10)
        qos_scan = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10)

        # --- Subscribers ---
        self.laser_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, qos_scan)

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.pose_topic, self.pose_callback, qos_profile)

        # --- Publisher for local costmap ---
        self.costmap_pub = self.create_publisher(
            OccupancyGrid, self.costmap_topic, qos_profile)

        # --- Timer to publish costmap at fixed rate ---
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_local_map)

        self.get_logger().info(f"✅ LocalCostmapNode started. Publishing on {self.costmap_topic} in {self.costmap_frame_id} frame.")

    # ---------------- Subscription Callbacks ----------------

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Extract (x, y, yaw) from the robot pose and update the TF transform."""
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = self.quaternion_to_yaw(qx, qy, qz, qw)
        self.robot_pose = (px, py, yaw)

        # Publish TF transform to attach costmap to robot
        self.publish_tf_transform(px, py, yaw)

    def scan_callback(self, scan: LaserScan):
        """Update the local costmap with LiDAR data (fixed in the map frame)."""
        if self.robot_pose is None:
            return

        # Reset costmap (initialize all free space to 0)
        self.local_grid.fill(0)

        robot_x, robot_y, robot_yaw = self.robot_pose
        angle = scan.angle_min

        for r in scan.ranges:
            if r < scan.range_min or r > scan.range_max or math.isinf(r) or math.isnan(r):
                angle += scan.angle_increment
                continue

            # ✅ Convert LiDAR hit to **global map frame**
            obs_x = robot_x + r * math.cos(angle + robot_yaw)
            obs_y = robot_y + r * math.sin(angle + robot_yaw)
            angle += scan.angle_increment

            # ✅ Convert global coordinates to costmap indices
            cx, cy = self.world_to_grid(obs_x, obs_y, robot_x, robot_y)

            # ✅ Mark obstacle if within bounds
            if 0 <= cx < self.grid_size and 0 <= cy < self.grid_size:
                self.local_grid[cy, cx] = 100  # Obstacle

        # ✅ Apply inflation
        self.apply_inflation()

    # ---------------- Inflation Processing ----------------

    def apply_inflation(self):
        """Expands obstacles to create a safety buffer in the costmap."""
        inflation_cells = int(self.inflation_radius / self.resolution)  # Inflation size in cells
        structuring_element = np.ones((2 * inflation_cells + 1, 2 * inflation_cells + 1))

        # Convert obstacles to binary format for inflation
        binary_obstacles = (self.local_grid == 100)
        inflated_mask = binary_dilation(binary_obstacles, structure=structuring_element)

        # Apply inflation effect
        self.local_grid[inflated_mask] = 50  # Inflated area (gray in RViz)

    # ---------------- Publish Costmap ----------------

    def publish_local_map(self):
        """Convert local grid into `OccupancyGrid` and publish."""
        if self.local_grid is None or self.robot_pose is None:
            return

        occ_grid = OccupancyGrid()
        occ_grid.header.stamp = self.get_clock().now().to_msg()
        occ_grid.header.frame_id = self.costmap_frame_id  # Attaches to robot

        occ_grid.info.resolution = self.resolution
        occ_grid.info.width = self.grid_size
        occ_grid.info.height = self.grid_size
        occ_grid.info.origin.orientation.w = 1.0  # No rotation

        # ✅ Costmap always centers on the robot
        occ_grid.info.origin.position.x = self.robot_pose[0] - (self.local_size / 2.0)
        occ_grid.info.origin.position.y = self.robot_pose[1] - (self.local_size / 2.0)

        # ✅ Flatten data for ROS message
        occ_grid.data = self.local_grid.flatten().tolist()

        self.costmap_pub.publish(occ_grid)

    # ---------------- TF Broadcasting ----------------

    def publish_tf_transform(self, x, y, yaw):
        """Broadcast a transform from `map` to `local_costmap`."""
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = "map"
        tf_msg.child_frame_id = self.costmap_frame_id

        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = 0.0

        qx, qy, qz, qw = self.yaw_to_quaternion(yaw)
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf_msg)

    # ---------------- Helper Functions ----------------

    def world_to_grid(self, wx, wy, rx, ry):
        """Convert world coordinates to costmap indices."""
        cx = int((wx - rx + self.local_size / 2) / self.resolution)
        cy = int((wy - ry + self.local_size / 2) / self.resolution)
        return cx, cy


    def quaternion_to_yaw(self, qx, qy, qz, qw):
        """Convert quaternion to yaw (Euler angle)."""
        return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

    def yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion."""
        return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

def main(args=None):
    rclpy.init(args=args)
    node = LocalCostmapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

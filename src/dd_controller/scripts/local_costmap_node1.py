#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import math

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class LocalCostmapNode(Node):
    """
    A ROS2 node that creates a robot-centered local costmap:
      - The costmap is in the robot's frame (e.g. 'base_link').
      - If centered=True, the robot is visually in the middle of the grid.
      - LiDAR hits are projected into this local grid, so obstacles move with the robot in RViz.
    """

    def __init__(self):
        super().__init__('local_costmap_node')
        
        # -------------------- Parameters --------------------
        self.declare_parameter('local_width', 5.0)   # [m] how wide the costmap is
        self.declare_parameter('local_height', 5.0)  # [m] how tall the costmap is
        self.declare_parameter('resolution', 0.05)    # [m/cell]
        self.declare_parameter('centered', True)      # Place the robot in the center of the costmap?
        self.declare_parameter('publish_rate', 5.0)   # [Hz] how often to publish
        # Frame of the costmap. Usually 'base_link' or 'base_footprint'.
        self.declare_parameter('costmap_frame', 'base_link')

        # Retrieve param values
        self.local_width = self.get_parameter('local_width').value
        self.local_height = self.get_parameter('local_height').value
        self.resolution = self.get_parameter('resolution').value
        self.centered = self.get_parameter('centered').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.costmap_frame = self.get_parameter('costmap_frame').value

        # Derived grid size
        self.grid_width = int(self.local_width / self.resolution)
        self.grid_height = int(self.local_height / self.resolution)

        # Offsets if the robot is in the center
        if self.centered:
            self.offset_x = self.grid_width // 2
            self.offset_y = self.grid_height // 2
        else:
            self.offset_x = 0
            self.offset_y = 0

        # We'll store an occupancy grid internally:
        #   0 => free
        #   100 => occupied
        #   -1 => unknown (not used here, we just set everything to 0 or 100)
        self.local_grid = None

        # We only need the robot's yaw from PoseWithCovariance if we want to do advanced transforms.
        # However, if the LiDAR is in the same frame as the costmap, we can skip the pose entirely
        # or assume yaw=0. For demonstration, we still store the yaw but won't do a full transform.
        self.robot_yaw = 0.0

        # -------------------- QoS Setup --------------------
        qos_profile_lidar = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        qos_profile_map = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        qos_pose = QoSProfile(depth=10)

        # -------------------- Subscriptions --------------------
        # 1) LiDAR => /scan
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, qos_profile_lidar
        )
        # 2) Robot pose => /amcl_pose (optional if we want full transform)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            qos_pose
        )

        # -------------------- Publisher --------------------
        # We'll publish on /local_costmap
        self.local_map_pub = self.create_publisher(OccupancyGrid, '/local_costmap', qos_profile_map)

        # -------------------- Timer --------------------
        # Publish the local costmap at 'publish_rate' Hz
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_local_map)

        self.get_logger().info("LocalCostmapNode started. Creating a robot-centered local costmap in base_link frame.")

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """
        If needed, parse out the yaw. This example node doesn't do a full transform
        from a global frame to 'base_link' if we assume the LiDAR is already in base_link.
        But let's still store the yaw for completeness.
        """
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.robot_yaw = self.quaternion_to_yaw(qx, qy, qz, qw)
        # We won't do anything else with the pose in this simple example.

    def laser_callback(self, scan: LaserScan):
        """
        Re-initialize the local grid. For a 'robot-centered' costmap in base_link frame,
        we treat the robot as (0,0), so each LiDAR hit is (r*cos(angle), r*sin(angle)).
        We'll place the robot in the center of the grid if self.centered=True.
        """
        # 1) Re-initialize local grid to free => 0
        self.local_grid = np.zeros((self.grid_height, self.grid_width), dtype=np.int8)

        angle = scan.angle_min
        beam_count = len(scan.ranges)

        for r in scan.ranges:
            if r < scan.range_min or r > scan.range_max or math.isinf(r) or math.isnan(r):
                angle += scan.angle_increment
                continue

            # In base_link frame, the robot is at (0,0). If we want to incorporate self.robot_yaw
            # (like if the LiDAR is rotated or if we had a difference in frames), we could do:
            # obs_x = r * cos(self.robot_yaw + angle)
            # obs_y = r * sin(self.robot_yaw + angle)
            # But if LiDAR is aligned with base_link, we can just do:
            obs_x = r * math.cos(angle)
            obs_y = r * math.sin(angle)
            angle += scan.angle_increment

            # Convert (obs_x, obs_y) => costmap cell
            cx = int(obs_x / self.resolution) + self.offset_x
            cy = int(obs_y / self.resolution) + self.offset_y

            # Mark as occupied => 100 if in range
            if 0 <= cx < self.grid_width and 0 <= cy < self.grid_height:
                self.local_grid[cy, cx] = 100

    def publish_local_map(self):
        """Build an OccupancyGrid in base_link frame so it moves with the robot in RViz."""
        if self.local_grid is None:
            return

        occ = OccupancyGrid()
        occ.header.stamp = self.get_clock().now().to_msg()
        # The costmap is in the robot frame => 'base_link' or 'base_footprint'
        occ.header.frame_id = self.costmap_frame

        occ.info.resolution = self.resolution
        occ.info.width = self.grid_width
        occ.info.height = self.grid_height
        occ.info.origin.orientation.w = 1.0  # no rotation

        # If the robot is in the center => shift origin so (0,0) is bottom-left
        if self.centered:
            occ.info.origin.position.x = - (self.local_width / 2.0)
            occ.info.origin.position.y = - (self.local_height / 2.0)
        else:
            occ.info.origin.position.x = 0.0
            occ.info.origin.position.y = 0.0

        # Flatten
        occ.data = self.local_grid.flatten().tolist()

        self.local_map_pub.publish(occ)

    # --------------------------------------

    def quaternion_to_yaw(self, qx, qy, qz, qw):
        """Basic helper to convert quaternion to yaw in radians."""
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = LocalCostmapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

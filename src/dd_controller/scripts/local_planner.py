#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
import heapq

from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Quaternion
from rclpy.qos import QoSProfile

class LocalReplannerNode(Node):
    """
    A node that:
      - Subscribes to /global_path (the high-level path from your global A* planner),
      - Subscribes to /local_costmap (occupancy grid),
      - Subscribes to robot pose (e.g. /amcl_pose),
      - Identifies a sub-goal in the local costmap region from the global path,
      - Runs A* in the local costmap to produce a collision-free local path,
      - Publishes that local path on /local_path for your pure pursuit node to follow.
    """

    def __init__(self):
        super().__init__('local_replanner')

        # ---------- Parameters ----------
        self.declare_parameter('global_path_topic', '/global_path')
        self.declare_parameter('local_costmap_topic', '/local_costmap')
        self.declare_parameter('pose_topic', '/amcl_pose')
        self.declare_parameter('local_path_topic', '/local_path')
        self.declare_parameter('plan_rate', 100.0)   # [Hz], how often to replan

        self.global_path_topic = self.get_parameter('global_path_topic').value
        self.local_costmap_topic = self.get_parameter('local_costmap_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.local_path_topic = self.get_parameter('local_path_topic').value
        self.plan_rate = self.get_parameter('plan_rate').value

        # ---------- State ----------
        self.global_path = []    # store the entire global path as (x,y) points
        self.local_costmap = None
        self.local_costmap_info = None
        self.robot_pose = None   # (rx, ry, yaw)

        # ---------- QoS & Subscriptions ----------
        qos = QoSProfile(depth=10)
        self.global_path_sub = self.create_subscription(Path, self.global_path_topic, self.global_path_callback, qos)
        self.costmap_sub = self.create_subscription(OccupancyGrid, self.local_costmap_topic, self.costmap_callback, qos)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, self.pose_topic, self.pose_callback, qos)

        # ---------- Publisher ----------
        self.local_path_pub = self.create_publisher(Path, self.local_path_topic, qos)

        # ---------- Timer for Re-planning ----------
        self.plan_timer = self.create_timer(1.0/self.plan_rate, self.plan_local_path)

        self.get_logger().info("LocalReplannerNode started. Publishes /local_path from /global_path + /local_costmap")

    # ---------- Callbacks ----------
    def global_path_callback(self, msg: Path):
        self.global_path = []
        for pose_stamped in msg.poses:
            px = pose_stamped.pose.position.x
            py = pose_stamped.pose.position.y
            self.global_path.append((px, py))
        self.get_logger().info(f"Received global path with {len(self.global_path)} waypoints.")

    def costmap_callback(self, msg: OccupancyGrid):
        w = msg.info.width
        h = msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        self.local_costmap = data
        self.local_costmap_info = msg.info

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        rx = msg.pose.pose.position.x
        ry = msg.pose.pose.position.y
        # yaw optional
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = self.quaternion_to_yaw(qx, qy, qz, qw)
        self.robot_pose = (rx, ry, yaw)

    # ---------- Timer-based A* Replanning ----------
    def plan_local_path(self):
        """Periodically plan a local path using A* from the robot's cell to a chosen sub-goal cell from global_path."""
        if self.local_costmap is None or not self.global_path or self.robot_pose is None:
            return

        # 1) Find sub-goal from global path within the local costmap region
        sub_goal = self.select_sub_goal()
        if sub_goal is None:
            self.get_logger().warn("No suitable sub-goal found in local costmap. Not publishing local path.")
            return

        # 2) Convert robot pose and sub-goal to costmap cells
        rx, ry, _ = self.robot_pose
        start_cx, start_cy = self.world_to_grid(rx, ry)
        goal_cx, goal_cy = self.world_to_grid(sub_goal[0], sub_goal[1])
        if not self.cell_in_bounds(start_cx, start_cy) or not self.cell_in_bounds(goal_cx, goal_cy):
            self.get_logger().warn("Robot or sub-goal is out of local costmap bounds. Not planning.")
            return

        # 3) A* search
        path_cells = self.a_star(start_cx, start_cy, goal_cx, goal_cy)
        if not path_cells:
            self.get_logger().warn("No local path found by A*. Possibly blocked.")
            return

        # 4) Convert cell path -> real coords -> nav_msgs/Path
        path_coords = [self.grid_to_world(cx, cy) for (cx, cy) in path_cells]
        path_msg = self.build_path_msg(path_coords)
        self.local_path_pub.publish(path_msg)
        self.get_logger().info(f"Local path with {len(path_coords)} waypoints published.")

    def select_sub_goal(self):
        """
        Find a sub-goal from the global path that is within the local costmap region or near it.
        Simple approach: pick the final global point that still lies inside the local costmap bounding box.
        If none is inside, return None.
        """
        if not self.local_costmap_info:
            return None

        # costmap boundaries in world coords
        origin_x = self.local_costmap_info.origin.position.x
        origin_y = self.local_costmap_info.origin.position.y
        w = self.local_costmap_info.width
        h = self.local_costmap_info.height
        res = self.local_costmap_info.resolution

        max_x = origin_x + w * res
        max_y = origin_y + h * res

        # We'll pick the last waypoint in the global path that is within costmap bounding box
        # bounding box: [origin_x, max_x] × [origin_y, max_y]
        chosen = None
        for (px, py) in self.global_path:
            if (px >= origin_x and px <= max_x and py >= origin_y and py <= max_y):
                chosen = (px, py)
        # If we found at least one, chosen will be the last such point
        return chosen

    # ---------- A* Implementation ----------
    def a_star(self, start_x, start_y, goal_x, goal_y):
        """A simple 8-connected A* in local_costmap. Occupied cells >=100 => blocked."""
        open_set = []
        heapq.heappush(open_set, (0, (start_x, start_y)))
        came_from = {}
        cost_so_far = {(start_x, start_y): 0}
        neighbors_8 = [(-1, 0), (1, 0), (0, -1), (0, 1),
                       (-1, -1), (-1, 1), (1, -1), (1, 1)]

        def heuristic(a, b):
            return math.hypot(b[0]-a[0], b[1]-a[1])

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == (goal_x, goal_y):
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append((start_x, start_y))
                path.reverse()
                return path

            for (dx, dy) in neighbors_8:
                nx = current[0] + dx
                ny = current[1] + dy
                if not self.cell_in_bounds(nx, ny):
                    continue
                if self.local_costmap[ny, nx] >= 100:
                    # Occupied
                    continue
                move_cost = math.hypot(dx, dy)
                new_cost = cost_so_far[current] + move_cost
                if (nx, ny) not in cost_so_far or new_cost < cost_so_far[(nx, ny)]:
                    cost_so_far[(nx, ny)] = new_cost
                    priority = new_cost + heuristic((nx, ny), (goal_x, goal_y))
                    heapq.heappush(open_set, (priority, (nx, ny)))
                    came_from[(nx, ny)] = current

        return None

    # ---------- Utils ----------
    def cell_in_bounds(self, cx, cy):
        if 0 <= cx < self.local_costmap_info.width and 0 <= cy < self.local_costmap_info.height:
            return True
        return False

    def world_to_grid(self, wx, wy):
        """Convert (wx, wy) in 'map' coords -> costmap cell indices."""
        ox = self.local_costmap_info.origin.position.x
        oy = self.local_costmap_info.origin.position.y
        r = self.local_costmap_info.resolution
        cx = int((wx - ox) / r)
        cy = int((wy - oy) / r)
        return (cx, cy)

    def grid_to_world(self, cx, cy):
        """Convert costmap cell -> center in 'map' coords."""
        ox = self.local_costmap_info.origin.position.x
        oy = self.local_costmap_info.origin.position.y
        r = self.local_costmap_info.resolution
        wx = (cx + 0.5)*r + ox
        wy = (cy + 0.5)*r + oy
        return (wx, wy)

    def build_path_msg(self, coords):
        """Convert list of (x,y) to nav_msgs/Path in costmap's frame_id."""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        # Use local costmap's frame (often "map" if your costmap is map-based)
        path_msg.header.frame_id = self.local_costmap_info.header.frame_id if hasattr(self.local_costmap_info, 'header') else 'map'

        for (x, y) in coords:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            # orientation= no particular heading
            ps.pose.orientation = Quaternion(w=1.0)
            path_msg.poses.append(ps)

        return path_msg

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        rx = msg.pose.pose.position.x
        ry = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = self.quaternion_to_yaw(qx, qy, qz, qw)
        self.robot_pose = (rx, ry, yaw)

    def quaternion_to_yaw(self, qx, qy, qz, qw):
        siny_cosp = 2.0*(qw*qz + qx*qy)
        cosy_cosp = 1.0 - 2.0*(qy*qy + qz*qz)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = LocalReplannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

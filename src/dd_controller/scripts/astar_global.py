#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PointStamped, Pose, Quaternion, PoseWithCovarianceStamped
from rclpy.qos import QoSProfile
import numpy as np
import math
import heapq

class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('global_planner_node')
        qos_profile = QoSProfile(depth=10)
        
        # Subscribers for costmap, current robot pose, and clicked point goal
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap', self.costmap_callback, qos_profile)
        self.start_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.start_callback, qos_profile)
        self.clicked_point_sub = self.create_subscription(
            PointStamped, '/clicked_point', self.clicked_point_callback, qos_profile)
        
        # Publisher for the computed global path
        self.path_pub = self.create_publisher(Path, '/global_path', qos_profile)
        
        # Internal state variables
        self.costmap = None
        self.costmap_info = None
        self.start_pose = None
        self.goal_pose = None

    def costmap_callback(self, msg: OccupancyGrid):
        width = msg.info.width
        height = msg.info.height
        grid_data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        self.costmap = grid_data
        self.costmap_info = msg.info
        self.get_logger().info("Costmap received")
        self.try_planning()

    def start_callback(self, msg: PoseWithCovarianceStamped):
        self.start_pose = msg.pose.pose
        self.get_logger().info("Start pose received")
        self.try_planning()

    def clicked_point_callback(self, msg: PointStamped):
        # Convert clicked point to a goal pose with default orientation
        goal_pose = Pose()
        goal_pose.position = msg.point
        goal_pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        self.goal_pose = goal_pose
        self.get_logger().info("Goal pose received from clicked_point")
        self.try_planning()

    def try_planning(self):
        if self.costmap is not None and self.start_pose is not None and self.goal_pose is not None:
            self.get_logger().info("All inputs received, starting A* planning")
            path = self.a_star_planning()
            if path:
                self.publish_path(path)
            else:
                self.get_logger().error("No path found")

    def a_star_planning(self):
        resolution = self.costmap_info.resolution
        origin_x = self.costmap_info.origin.position.x
        origin_y = self.costmap_info.origin.position.y
        width = self.costmap_info.width
        height = self.costmap_info.height

        def world_to_grid(x, y):
            gx = int((x - origin_x) / resolution)
            gy = int((y - origin_y) / resolution)
            return gx, gy

        def grid_to_world(gx, gy):
            x = gx * resolution + origin_x + resolution / 2.0
            y = gy * resolution + origin_y + resolution / 2.0
            return x, y

        start_x, start_y = world_to_grid(self.start_pose.position.x, self.start_pose.position.y)
        goal_x, goal_y = world_to_grid(self.goal_pose.position.x, self.goal_pose.position.y)

        if not (0 <= start_x < width and 0 <= start_y < height):
            self.get_logger().error("Start position is out of costmap bounds")
            return None
        if not (0 <= goal_x < width and 0 <= goal_y < height):
            self.get_logger().error("Goal position is out of costmap bounds")
            return None

        open_set = []
        heapq.heappush(open_set, (0, (start_x, start_y)))
        came_from = {}
        cost_so_far = {(start_x, start_y): 0}

        def heuristic(a, b):
            return math.hypot(b[0] - a[0], b[1] - a[1])

        neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1),
                     (-1, -1), (-1, 1), (1, -1), (1, 1)]

        while open_set:
            current_priority, current = heapq.heappop(open_set)

            if current == (goal_x, goal_y):
                path = []
                while current != (start_x, start_y):
                    path.append(current)
                    current = came_from[current]
                path.append((start_x, start_y))
                path.reverse()
                path_world = [grid_to_world(x, y) for (x, y) in path]
                return path_world

            for dx, dy in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)
                nx, ny = neighbor
                if 0 <= nx < width and 0 <= ny < height:
                    if self.costmap[ny, nx] >= 100:
                        continue
                    move_cost = math.hypot(dx, dy)
                    new_cost = cost_so_far[current] + move_cost + self.costmap[ny, nx] / 100.0
                    if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                        cost_so_far[neighbor] = new_cost
                        priority = new_cost + heuristic(neighbor, (goal_x, goal_y))
                        heapq.heappush(open_set, (priority, neighbor))
                        came_from[neighbor] = current
        return None

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        for (x, y) in path:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
            path_msg.poses.append(pose_stamped)
        self.path_pub.publish(path_msg)
        self.get_logger().info("Published global path with {} poses".format(len(path_msg.poses)))

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import numpy as np
from PIL import Image
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from scipy.ndimage import binary_dilation
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class Costmap:
    def __init__(self, yaml_file, robot_radius=0.6, safety_margin=0.1):
        with open(yaml_file, 'r') as file:
            map_data = yaml.safe_load(file)

     
        my_ws_dir = get_package_share_directory('dd_controller')
        pgm_path = os.path.join(my_ws_dir, 'map', 'map.pgm')
        pgm_image = Image.open(pgm_path)
        self.grid = np.array(pgm_image)
        self.grid = np.flipud(self.grid)

  
        self.resolution = map_data['resolution']
        self.origin_x = map_data['origin'][0]
        self.origin_y = map_data['origin'][1]
        self.occupied_thresh = map_data['occupied_thresh']
        self.free_thresh = map_data['free_thresh']

        # Inflate obstacles based on robot radius and safety margin
        self.robot_radius = robot_radius
        self.safety_margin = safety_margin
        self.inflate_obstacles()

    def inflate_obstacles(self):
        """
        Inflate obstacles in the costmap by the robot's radius and safety margin.
        """
        # Create a binary mask of obstacles from the inverted image
        inverted_image = 255 - self.grid
        obstacle_mask = (inverted_image / 255.0) > self.occupied_thresh

        # Calculate the inflation radius in cells using an inflation factor
        inflation_factor = 1.5  # increase this factor to expand inflation further
        inflation_radius = int(((self.robot_radius + self.safety_margin) / self.resolution) * inflation_factor)

        # Create a circular structuring element for dilation
        y, x = np.ogrid[-inflation_radius:inflation_radius + 1, -inflation_radius:inflation_radius + 1]
        structuring_element = x**2 + y**2 <= inflation_radius**2

        # Dilate the obstacle mask
        inflated_mask = binary_dilation(obstacle_mask, structure=structuring_element)

        # Update the grid with inflated obstacles using ROS occupancy convention (100 = occupied)
        self.grid[inflated_mask] = 100

    def get_cost(self, x, y):
        """
        Get the cost of a cell at (x, y) in world coordinates.
        """
        cell_x = int((x - self.origin_x) / self.resolution)
        cell_y = int((y - self.origin_y) / self.resolution)
        if 0 <= cell_x < self.grid.shape[1] and 0 <= cell_y < self.grid.shape[0]:
            pixel_value = self.grid[cell_y, cell_x] / 255.0  # Normalize to [0, 1]
            if pixel_value > self.occupied_thresh:
                return 100  # Occupied
            elif pixel_value < self.free_thresh:
                return 0    # Free
            else:
                return 50   # Unknown
        return 100  # Treat out-of-bounds as occupied

class CostmapNode(Node):
    def __init__(self):
        super().__init__('costmap_node')

        
        my_ws_dir = get_package_share_directory('dd_controller')
        map_yaml_file = os.path.join(my_ws_dir, 'map', 'map.yaml')

        self.declare_parameter('yaml_file', map_yaml_file)
        yaml_file = self.get_parameter('yaml_file').value

        # Initialize costmap 
        self.costmap = Costmap(yaml_file, robot_radius=0.1, safety_margin=0.2)

    
        qos_map= QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10)


       
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/global_costmap', qos_map)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)

        
        self.timer = self.create_timer(1.0, self.publish_costmap)

    def pose_callback(self, msg):
        
        self.robot_pose = msg.pose.pose

    def publish_costmap(self):
        
        costmap_msg = OccupancyGrid()
        costmap_msg.header.stamp = self.get_clock().now().to_msg()
        costmap_msg.header.frame_id = 'map'
        costmap_msg.info.resolution = self.costmap.resolution
        costmap_msg.info.width = self.costmap.grid.shape[1]
        costmap_msg.info.height = self.costmap.grid.shape[0]
        costmap_msg.info.origin.position.x = self.costmap.origin_x
        costmap_msg.info.origin.position.y = self.costmap.origin_y

        
        costmap_data = self.costmap.grid.astype(np.int8).flatten().tolist()
        costmap_msg.data = costmap_data

        self.costmap_pub.publish(costmap_msg)

def main(args=None):
    rclpy.init(args=args)
    costmap_node = CostmapNode()
    rclpy.spin(costmap_node)
    costmap_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

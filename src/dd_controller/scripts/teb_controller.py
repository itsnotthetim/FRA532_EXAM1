#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.qos import QoSProfile

class TEBLocalPlannerCostmapNode(Node):
    """
    TEB local planner using an occupancy grid for collision checks.
    - Subscribes: /local_costmap (OccupancyGrid), /amcl_pose (robot pose).
    - Maintains a 'band' of states from robot pose -> sub-goal_x,y.
    - Runs a simple iterative 'toy' optimization each cycle to push states from obstacles.
    - Publishes velocity commands on /cmd_vel, plus the final band as /local_path for visualization.
    """

    def __init__(self):
        super().__init__('teb_local_planner_costmap')

        # ---- Parameters ----
        self.declare_parameter('sub_goal_x', -1.0)
        self.declare_parameter('sub_goal_y', -1.0)
        self.declare_parameter('num_states', 10)
        self.declare_parameter('time_horizon', 2.0)
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('max_yaw_rate', 1.0)
        self.declare_parameter('control_rate', 5.0)
        self.declare_parameter('costmap_topic', '/local_costmap')
        self.declare_parameter('pose_topic', '/amcl_pose')

        self.sub_goal_x = self.get_parameter('sub_goal_x').value
        self.sub_goal_y = self.get_parameter('sub_goal_y').value
        self.num_states = self.get_parameter('num_states').value
        self.time_horizon = self.get_parameter('time_horizon').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').value
        self.control_rate = self.get_parameter('control_rate').value
        self.costmap_topic = self.get_parameter('costmap_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value

        # State
        self.local_costmap = None
        self.local_costmap_info = None
        self.robot_pose = (0.0, 0.0, 0.0)
        self.band = []

        qos_profile = QoSProfile(depth=10)
        # Subscriptions
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, self.costmap_topic, self.costmap_callback, qos_profile)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.pose_topic, self.pose_callback, qos_profile)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.local_path_pub = self.create_publisher(Path, '/local_path', qos_profile)

        # Timer
        self.timer = self.create_timer(1.0/self.control_rate, self.teb_control_loop)

        self.get_logger().info("TEBLocalPlannerCostmapNode started. Sub-goal=({:.2f},{:.2f})".format(
            self.sub_goal_x, self.sub_goal_y))

    # -------------- Callbacks --------------
    def costmap_callback(self, msg: OccupancyGrid):
        w = msg.info.width
        h = msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        self.local_costmap = data
        self.local_costmap_info = msg.info

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = self.quaternion_to_yaw(qx, qy, qz, qw)
        self.robot_pose = (px, py, yaw)

    # -------------- TEB Control Loop --------------
    def teb_control_loop(self):
        if self.local_costmap is None or self.local_costmap_info is None:
            return
        rx, ry, ryaw = self.robot_pose

        # If band not init or size mismatch, init
        if not self.band or len(self.band)!=self.num_states:
            self.initialize_band(rx, ry, ryaw)

        # simple iteration
        for _ in range(5):
            self.optimize_band()

        # compute velocity from first 2 states
        if len(self.band)>=2:
            x0, y0, yaw0, t0 = self.band[0]
            x1, y1, yaw1, t1 = self.band[1]
            dt = (t1 - t0) if (t1>t0) else self.time_horizon/self.num_states
            vx = (x1 - x0)/dt
            vy = (y1 - y0)/dt
            v_f = math.hypot(vx, vy)
            yaw_diff = self.angle_diff(yaw1, yaw0)
            omega = yaw_diff/dt

            # clamp
            if v_f>self.max_speed:
                v_f = self.max_speed
            if abs(omega)>self.max_yaw_rate:
                omega = math.copysign(self.max_yaw_rate, omega)

            # publish
            tw = Twist()
            tw.linear.x = v_f
            tw.angular.z = omega
            self.cmd_vel_pub.publish(tw)

        # publish path
        self.publish_local_path()

    def initialize_band(self, rx, ry, ryaw):
        self.band = []
        dt = self.time_horizon/(self.num_states-1)
        for i in range(self.num_states):
            alpha = i/(self.num_states-1)
            t = i*dt
            x = rx + alpha*(self.sub_goal_x - rx)
            y = ry + alpha*(self.sub_goal_y - ry)
            # keep yaw constant
            self.band.append((x,y,ryaw,t))

    def optimize_band(self):
        if not self.band:
            return
        for i in range(1, len(self.band)-1):
            x, y, yaw, t = self.band[i]
            dx, dy = self.obstacle_gradient(x, y)
            xprev, yprev, _, _ = self.band[i-1]
            xnext, ynext, _, _ = self.band[i+1]
            xm = 0.5*(xprev + xnext)
            ym = 0.5*(yprev + ynext)
            # sub goal pull
            w_goal = 0.01
            gx = w_goal*(self.sub_goal_x - x)
            gy = w_goal*(self.sub_goal_y - y)

            new_x = x + 0.1*(-dx + 0.5*(xm - x) + gx)
            new_y = y + 0.1*(-dy + 0.5*(ym - y) + gy)
            # keep yaw
            self.band[i] = (new_x, new_y, yaw, t)

    def obstacle_gradient(self, x, y):
        if self.local_costmap is None:
            return (0.0, 0.0)
        ox = self.local_costmap_info.origin.position.x
        oy = self.local_costmap_info.origin.position.y
        r = self.local_costmap_info.resolution
        w = self.local_costmap_info.width
        h = self.local_costmap_info.height

        cx = int((x-ox)/r)
        cy = int((y-oy)/r)
        if cx<0 or cx>=w or cy<0 or cy>=h:
            return (0.0,0.0)

        max_r = 4
        best_dist = float('inf')
        best_dx = 0
        best_dy = 0
        for dx in range(-max_r, max_r+1):
            for dy in range(-max_r, max_r+1):
                nx = cx+dx
                ny = cy+dy
                if nx<0 or nx>=w or ny<0 or ny>=h:
                    continue
                if self.local_costmap[ny,nx]>=100:
                    dist_sq = dx*dx+dy*dy
                    if dist_sq<best_dist:
                        best_dist = dist_sq
                        best_dx = dx
                        best_dy = dy
        if best_dist==float('inf'):
            return (0.0,0.0)
        dist = math.sqrt(best_dist)
        if dist<1e-3:
            dist=1e-3
        gx = -best_dx/dist
        gy = -best_dy/dist
        scale = 1.0/dist
        return (gx*scale, gy*scale)

    def publish_local_path(self):
        from nav_msgs.msg import Path
        from geometry_msgs.msg import PoseStamped, Quaternion
        if not self.band:
            return
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        frame_id = 'map'
        if hasattr(self.local_costmap_info, 'header'):
            frame_id = self.local_costmap_info.header.frame_id
        path_msg.header.frame_id = frame_id

        for (x,y,yaw,t) in self.band:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            # orientation
            qz = math.sin(yaw/2.0)
            qw = math.cos(yaw/2.0)
            ps.pose.orientation = Quaternion(z=qz, w=qw)
            path_msg.poses.append(ps)

        self.local_path_pub.publish(path_msg)

    def angle_diff(self, a,b):
        d = a-b
        while d>math.pi:
            d-=2.0*math.pi
        while d<-math.pi:
            d+=2.0*math.pi
        return d

    def quaternion_to_yaw(self, qx,qy,qz,qw):
        siny_cosp = 2.0*(qw*qz + qx*qy)
        cosy_cosp = 1.0 - 2.0*(qy*qy + qz*qz)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = TEBLocalPlannerCostmapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

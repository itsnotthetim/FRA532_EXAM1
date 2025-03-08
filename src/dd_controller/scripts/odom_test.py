import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class MyController(Node):
    def __init__(self):
        super().__init__('my_controller')
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )
        self.get_logger().info("Subscribed to /amcl_pose")

    def amcl_pose_callback(self, msg):
        # Use the robot's estimated pose for planning
        robot_pose = msg.pose.pose
        self.get_logger().info(f"Robot Pose: {robot_pose}")

def main(args=None):
    rclpy.init(args=args)
    controller = MyController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
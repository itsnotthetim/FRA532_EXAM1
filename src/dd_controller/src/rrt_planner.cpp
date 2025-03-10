#include "nav2_core/controller.hpp"
#include "nav2_util/node_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "nav_msgs/msg/path.hpp"

namespace my_rrt_planner
{

class MyRRTPlanner : public nav2_core::Controller
{
public:
  MyRRTPlanner() = default;
  ~MyRRTPlanner() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    node_ = parent.lock();
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    name_ = name;

    // Load parameters
    nav2_util::declare_parameter_if_not_declared(
      node_, name + ".max_iterations", rclcpp::ParameterValue(1000));
    node_->get_parameter(name + ".max_iterations", max_iterations_);

    nav2_util::declare_parameter_if_not_declared(
      node_, name + ".step_size", rclcpp::ParameterValue(0.1));
    node_->get_parameter(name + ".step_size", step_size_);

    RCLCPP_INFO(node_->get_logger(), "MyRRTPlanner configured!");
  }

  void activate() override
  {
    RCLCPP_INFO(node_->get_logger(), "MyRRTPlanner activated!");
  }

  void deactivate() override
  {
    RCLCPP_INFO(node_->get_logger(), "MyRRTPlanner deactivated!");
  }

  void cleanup() override
  {
    RCLCPP_INFO(node_->get_logger(), "MyRRTPlanner cleaned up!");
  }

  void setPlan(const nav_msgs::msg::Path & path) override
  {
    // Store the global path
    global_path_ = path;
    RCLCPP_INFO(node_->get_logger(), "New global path set with %zu poses!", global_path_.poses.size());
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override
  {
    geometry_msgs::msg::TwistStamped cmd_vel;

    // Debug log to verify costmap data
    if (costmap_ros_) {
      auto costmap = costmap_ros_->getCostmap();
      RCLCPP_INFO(node_->get_logger(), "Costmap size: %d x %d", costmap->getSizeInCellsX(), costmap->getSizeInCellsY());
    }

    // Placeholder logic for RRT-based planning
    // For now, just return a zero velocity command
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;

    RCLCPP_INFO(node_->get_logger(), "Computed velocity command!");
    return cmd_vel;
  }

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override
  {
    // Set the speed limit
    speed_limit_ = speed_limit;
    RCLCPP_INFO(node_->get_logger(), "Speed limit set to %f", speed_limit_);
  }

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_;

  // RRT parameters
  int max_iterations_;
  double step_size_;

  // Global path
  nav_msgs::msg::Path global_path_;

  // Speed limit
  double speed_limit_;
};

}  // namespace my_rrt_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_rrt_planner::MyRRTPlanner, nav2_core::Controller)
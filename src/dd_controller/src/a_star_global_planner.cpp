#include <pluginlib/class_list_macros.hpp>
#include <nav2_core/global_planner.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <vector>

namespace a_star_global_planner
{

class AStarGlobalPlanner : public nav2_core::GlobalPlanner
{
public:
  AStarGlobalPlanner() = default;
  ~AStarGlobalPlanner() = default;

  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
      std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    node_ = parent.lock();
    if (!node_) {
      throw std::runtime_error("Failed to lock node in A* Global Planner");
    }
    name_ = name;
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();

    RCLCPP_INFO(node_->get_logger(), "A* Global Planner configured");
  }

  void cleanup() override {
    RCLCPP_INFO(node_->get_logger(), "Cleaning up A* Global Planner");
  }

  void activate() override {
    RCLCPP_INFO(node_->get_logger(), "Activating A* Global Planner");
  }

  void deactivate() override {
    RCLCPP_INFO(node_->get_logger(), "Deactivating A* Global Planner");
  }

  nav_msgs::msg::Path createPlan(
      const geometry_msgs::msg::PoseStamped &start,
      const geometry_msgs::msg::PoseStamped &goal) override
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = node_->now();

    // Convert start and goal poses to costmap coordinates
    unsigned int start_x, start_y;
    unsigned int goal_x, goal_y;
    if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y) ||
        !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
      RCLCPP_ERROR(node_->get_logger(), "Start or goal pose is outside the costmap");
      return path;
    }

    // Run A* algorithm
    std::vector<std::pair<int, int>> path_coords = aStar(start_x, start_y, goal_x, goal_y);

    // Convert path coordinates to world coordinates
    for (const auto &coord : path_coords) {
      double wx, wy;
      costmap_->mapToWorld(coord.first, coord.second, wx, wy);

      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = wx;
      pose.pose.position.y = wy;
      pose.pose.orientation.w = 1.0; // Neutral orientation
      path.poses.push_back(pose);
    }

    RCLCPP_INFO(node_->get_logger(), "Path planned with %zu points", path.poses.size());
    return path;
  }

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D *costmap_;

  // A* algorithm implementation
  std::vector<std::pair<int, int>> aStar(unsigned int start_x, unsigned int start_y, unsigned int goal_x, unsigned int goal_y)
  {
    // Define a node structure for A*
    struct Node {
      int x, y;
      double g, h;
      Node* parent;

      Node(int x, int y, double g, double h, Node* parent = nullptr)
        : x(x), y(y), g(g), h(h), parent(parent) {}

      double f() const { return g + h; }
    };

    // Comparator for priority queue
    auto cmp = [](const Node* a, const Node* b) { return a->f() > b->f(); };
    std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> open_set(cmp);

    // Hash map to store visited nodes
    std::unordered_map<int, Node*> all_nodes;

    // Start node
    Node* start_node = new Node(start_x, start_y, 0, heuristic(start_x, start_y, goal_x, goal_y));
    open_set.push(start_node);
    all_nodes[start_x * costmap_->getSizeInCellsY() + start_y] = start_node;

    // Directions for 8-connected grid
    const int dx[] = {1, -1, 0, 0, 1, -1, 1, -1};
    const int dy[] = {0, 0, 1, -1, 1, -1, -1, 1};

    while (!open_set.empty()) {
      Node* current = open_set.top();
      open_set.pop();

      // Check if current node is the goal
      if (current->x == static_cast<int>(goal_x) && current->y == static_cast<int>(goal_y)) {
        std::vector<std::pair<int, int>> path;
        while (current) {
          path.emplace_back(current->x, current->y);
          current = current->parent;
        }
        std::reverse(path.begin(), path.end());

        // Clean up allocated nodes
        for (auto& node : all_nodes) {
          delete node.second;
        }
        return path;
      }

      // Explore neighbors
      for (int i = 0; i < 8; ++i) {
        int nx = current->x + dx[i];
        int ny = current->y + dy[i];

        // Check if neighbor is within bounds and not an obstacle
        if (nx >= 0 && nx < static_cast<int>(costmap_->getSizeInCellsX()) &&
            ny >= 0 && ny < static_cast<int>(costmap_->getSizeInCellsY()) &&
            costmap_->getCost(nx, ny) < nav2_costmap_2d::LETHAL_OBSTACLE) {

          double g = current->g + (i < 4 ? 1.0 : 1.414); // Cost for 4-connected vs 8-connected
          double h = heuristic(nx, ny, goal_x, goal_y);
          int key = nx * costmap_->getSizeInCellsY() + ny;

          if (all_nodes.find(key) == all_nodes.end() || g < all_nodes[key]->g) {
            Node* neighbor = new Node(nx, ny, g, h, current);
            open_set.push(neighbor);
            all_nodes[key] = neighbor;
          }
        }
      }
    }

    // Clean up allocated nodes
    for (auto& node : all_nodes) {
      delete node.second;
    }

    RCLCPP_WARN(node_->get_logger(), "A* failed to find a path");
    return {};
  }

  // Heuristic function (Euclidean distance)
  double heuristic(int x1, int y1, int x2, int y2) {
    return std::hypot(x2 - x1, y2 - y1);
  }
};

} // namespace a_star_global_planner

PLUGINLIB_EXPORT_CLASS(a_star_global_planner::AStarGlobalPlanner, nav2_core::GlobalPlanner)
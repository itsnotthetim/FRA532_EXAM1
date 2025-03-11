## SBMPO Controller Node - README

### Overview
The **SBMPO (Sampling-Based Model Predictive Optimization) Controller Node** is a ROS2 implementation of a **local planner and controller** that generates **collision-free trajectories** for a mobile robot. It samples multiple candidate velocities, simulates their trajectories, and selects the optimal one based on a **cost function**.

This node:
- **Subscribes to:**
  - `/amcl_pose` → Robot’s current position.
  - `/global_path` → Path provided by the global planner.
  - `/local_costmap` → Real-time costmap for obstacle avoidance.
- **Publishes:**
  - `/cmd_vel` → Safe velocity commands for robot motion.
  - `/local_path` → Visual representation of the best trajectory.
- **Uses a model-predictive approach** to generate optimal motion commands while avoiding obstacles dynamically.

---

### Algorithm
#### **1. Motion Prediction and Trajectory Sampling**
The planner **samples multiple linear (`v`) and angular (`w`) velocities** and predicts their future motion using the kinematic model:

$$
    x_{t+1} = x_t + v \cos(\theta_t) \cdot dt
$$
$$
    y_{t+1} = y_t + v \sin(\theta_t) \cdot dt
$$
$$
    \theta_{t+1} = \theta_t + w \cdot dt
$$
where:
- \( x_t, y_t, \theta_t \) → Current robot position and heading angle.
- \( v \) → Linear velocity (m/s).
- \( w \) → Angular velocity (rad/s).
- \( dt \) → Simulation time step.

Each trajectory is evaluated over a **horizon time (`horizon`)** to determine its feasibility and cost.

#### **2. SBMPO Cost Function for Trajectory Selection**
Each trajectory candidate is evaluated using the **SBMPO cost function**:
$$
    C_{total} = C_{obs} + C_{path} - R_{forward} + C_{clearance}
$$
where:
-  $C_{obs} = W_{obs} \times \frac{cost(x, y)}{100}$  → **Obstacle proximity penalty**: Increases cost for paths close to obstacles.
-  $C_{path} = W_{path} \times d_{path}(x, y)$  → **Path deviation penalty**: Penalizes deviation from the planned global path.
-  $R_{forward} = W_{forward} \times v$  → **Forward motion reward**: Encourages the robot to move forward instead of stopping.
-  $C_{clearance}$  → **Low-clearance penalty**: Penalizes paths that navigate too close to obstacles, encouraging safer paths.

##### **Variable Definitions**:
| Variable         | Description |
|-----------------|-------------|
|  $C_{obs}$  | Obstacle proximity penalty; increases if the trajectory gets too close to obstacles. |
|  $W_{obs}$   | Weight coefficient for obstacle penalty (adjustable parameter). |
|  $cost(x, y)$  | Costmap value at position  $(x, y)$ , scaled to [0,100]. |
|  $C_{path}$  | Path deviation penalty; penalizes trajectories that deviate from the planned path. |
|  $W_{path}$  | Weight coefficient for path deviation (adjustable parameter). |
|  $d_{path}(x, y)$  | Euclidean distance from trajectory point  $(x, y)$  to the closest global path point. |
|  $R_{forward}$  | Forward motion reward; encourages the robot to move forward. |
|  $W_{forward}$  | Weight coefficient for forward reward (adjustable parameter). |
|  $v $ | Linear velocity of the trajectory candidate. |
|  $C_{clearance}$  | Clearance penalty; penalizes paths with insufficient obstacle clearance. |

If a trajectory results in a predicted collision within **emergency time-to-collision (`emergency_ttc`)**, it is immediately **discarded**.

#### **3. Trajectory Execution**
- The **lowest-cost trajectory** is selected and executed.
- If **no valid forward motion is found**, the controller attempts:
  1. **Reverse motion** (negative `v`).
  2. **Rotation in place** (zero `v`, maximum `w`).
  3. **Emergency stop** if no feasible option exists.

---

### ROS2 Implementation
#### **1. SBMPO Controller Class**
- Samples multiple **(v, w) trajectory candidates**.
- Predicts future robot motion.
- Evaluates trajectories using the **SBMPO cost function**.
- Selects the best trajectory and **publishes `/cmd_vel`**.

#### **2. SBMPO Controller Node**
- **Subscribes to:**
  - `/amcl_pose` → Current robot pose.
  - `/global_path` → Precomputed path from the global planner (to update the path).
  - `/local_costmap` → Real-time environment updates.
- **Publishes:**
  - `/cmd_vel` → Safe velocity command for robot motion.
  - `/local_path` → Visual representation of the selected trajectory.

---

### ROS2 Topics
| Topic              | Type                          | Role |
|--------------------|-----------------------------|------|
| `/amcl_pose`       | `PoseWithCovarianceStamped` | Robot’s estimated position. |
| `/global_path`     | `Path`                       | Global path from the planner. |
| `/local_costmap`   | `OccupancyGrid`              | Local obstacle map. |
| `/cmd_vel`         | `Twist`                      | Published velocity commands. |
| `/local_path`      | `Path`                       | Planned local trajectory for visualization. |

---

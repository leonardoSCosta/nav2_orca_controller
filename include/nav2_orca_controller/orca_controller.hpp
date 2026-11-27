#ifndef NAV2_ORCA_CONTROLLER__ORCA_CONTROLLER_HPP_
#define NAV2_ORCA_CONTROLLER__ORCA_CONTROLLER_HPP_

#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
// RVO2
#include "RVO.h"

#include "nav2_orca_controller/obstacle_clusterer.hpp"

namespace nav2_orca_controller
{

/**
 * @class OrcaController
 * @brief Nav2 local controller plugin using RVO2 (ORCA) algorithm
 */
class OrcaController : public nav2_core::Controller
{
public:
  OrcaController() = default;
  ~OrcaController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  /**
   * @brief Compute differential-drive velocity commands with forward preference,
   * turn-in-place hysteresis, angular velocity clamping, and bounded short reversal.
   */
  void mapDiffDriveVelocity(
    double v_x_robot,
    double v_y_robot,
    const geometry_msgs::msg::Point & current_pos,
    const rclcpp::Time & now,
    geometry_msgs::msg::Twist & out_twist);

  /**
   * @brief Transform a Path into target_frame using the given tf_buffer.
   */
  static nav_msgs::msg::Path transformPlanToFrame(
    const nav_msgs::msg::Path & plan,
    const std::string & target_frame,
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
    const rclcpp::Logger & logger);

  /**
   * @brief Select lookahead pose from an already-transformed plan (in robot_pose's frame)
   * while advancing closest_plan_idx_ monotonically.
   */
  geometry_msgs::msg::PoseStamped getLookaheadPointFromPlan(
    const nav_msgs::msg::Path & transformed_plan,
    const geometry_msgs::msg::PoseStamped & robot_pose);

  /**
   * @brief Compute goal arrival command when within xy_goal_tolerance (with hysteresis).
   * Returns true if the robot is in the final goal alignment phase and out_twist was populated.
   */
  bool computeGoalArrivalCommand(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::PoseStamped & final_goal_pose,
    double xy_tolerance,
    double yaw_tolerance,
    geometry_msgs::msg::Twist & out_twist);

protected:
  geometry_msgs::msg::PoseStamped getLookaheadPoint(
    const geometry_msgs::msg::PoseStamped & robot_pose);

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("OrcaController")};
  rclcpp::Clock::SharedPtr clock_;

  nav_msgs::msg::Path global_plan_;
  size_t closest_plan_idx_{0};
  std::shared_ptr<ObstacleClusterer> obstacle_clusterer_;

  std::unique_ptr<RVO::RVOSimulator> sim_;

  // ORCA parameters
  double time_step_{0.1};
  double neighbor_dist_{5.0};
  int max_neighbors_{10};
  double time_horizon_{3.0};
  double time_horizon_obst_{1.5};
  double robot_radius_{0.22};
  double safety_buffer_{0.10};
  double max_speed_{0.5};
  double speed_limit_{0.5};

  // Controller specifics
  double lookahead_dist_{0.45};
  bool is_holonomic_{false};
  double kw_{1.8};                 // Angular gain for diff-drive
  double max_angular_vel_{0.8};    // Maximum rotational velocity (rad/s)
  bool debug_mode_{false};

  // Goal convergence parameters
  bool rotate_to_goal_heading_{true};
  double goal_dist_tolerance_{0.20};
  double goal_yaw_tolerance_{0.15};
  double goal_decel_distance_{0.50};
  double min_approach_speed_{0.06};
  bool xy_goal_reached_{false};

  // Forward preference & short reversal parameters
  bool allow_short_reversal_{true};
  double min_backward_angle_{2.44};          // ~140 degrees in rad
  double max_backward_distance_{0.35};       // meters
  double max_backward_duration_{1.5};        // seconds
  double max_backward_speed_{0.15};          // m/s
  double forward_turn_slowdown_angle_{1.22}; // ~70 degrees in rad

  // Reversal & turn-in-place state tracking
  bool is_reversing_{false};
  bool reverse_lockout_{false};
  bool in_place_turning_{false};
  rclcpp::Time reverse_start_time_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::Point reverse_start_pos_;

  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

  void publishMarkers(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::TwistStamped & cmd_vel,
    const std::vector<OrcaObstacle> & obstacles,
    const std::vector<OrcaPolygon> & polygons);
  void publishDiagnostics(double laser_time, double costmap_time, double compute_time);
};

}  // namespace nav2_orca_controller

#endif  // NAV2_ORCA_CONTROLLER__ORCA_CONTROLLER_HPP_

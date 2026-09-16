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
  std::shared_ptr<ObstacleClusterer> obstacle_clusterer_;

  std::unique_ptr<RVO::RVOSimulator> sim_;

  // ORCA parameters
  double time_step_;
  double neighbor_dist_;
  int max_neighbors_;
  double time_horizon_;
  double time_horizon_obst_;
  double robot_radius_;
  double safety_buffer_;
  double max_speed_;
  double speed_limit_;

  // Controller specifics
  double lookahead_dist_;
  bool is_holonomic_;
  double kw_; // Angular gain for diff-drive

  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  
  void publishMarkers(const geometry_msgs::msg::PoseStamped & robot_pose, const geometry_msgs::msg::TwistStamped & cmd_vel, const std::vector<OrcaObstacle> & obstacles);
  void publishDiagnostics(double laser_time, double costmap_time, double compute_time);
};

}  // namespace nav2_orca_controller

#endif  // NAV2_ORCA_CONTROLLER__ORCA_CONTROLLER_HPP_

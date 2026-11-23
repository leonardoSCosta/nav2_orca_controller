#include "nav2_orca_controller/orca_controller.hpp"
#include <cmath>
#include <algorithm>
#include "tf2/utils.h"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include <chrono>

using nav2_util::declare_parameter_if_not_declared;

namespace nav2_orca_controller
{

void OrcaController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  name_ = name;
  tf_buffer_ = tf;
  costmap_ros_ = costmap_ros;

  auto node = parent_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node in OrcaController");
  }
  clock_ = node->get_clock();

  // ORCA Parameters
  declare_parameter_if_not_declared(node, name_ + ".time_step", rclcpp::ParameterValue(0.1));
  node->get_parameter(name_ + ".time_step", time_step_);

  declare_parameter_if_not_declared(node, name_ + ".neighbor_dist", rclcpp::ParameterValue(5.0));
  node->get_parameter(name_ + ".neighbor_dist", neighbor_dist_);

  declare_parameter_if_not_declared(node, name_ + ".max_neighbors", rclcpp::ParameterValue(10));
  node->get_parameter(name_ + ".max_neighbors", max_neighbors_);

  declare_parameter_if_not_declared(node, name_ + ".time_horizon", rclcpp::ParameterValue(3.0));
  node->get_parameter(name_ + ".time_horizon", time_horizon_);

  declare_parameter_if_not_declared(node, name_ + ".time_horizon_obst", rclcpp::ParameterValue(1.5));
  node->get_parameter(name_ + ".time_horizon_obst", time_horizon_obst_);

  declare_parameter_if_not_declared(node, name_ + ".robot_radius", rclcpp::ParameterValue(0.22));
  node->get_parameter(name_ + ".robot_radius", robot_radius_);

  declare_parameter_if_not_declared(node, name_ + ".safety_buffer", rclcpp::ParameterValue(0.10));
  node->get_parameter(name_ + ".safety_buffer", safety_buffer_);

  declare_parameter_if_not_declared(node, name_ + ".max_speed", rclcpp::ParameterValue(0.5));
  node->get_parameter(name_ + ".max_speed", max_speed_);

  speed_limit_ = max_speed_;

  // Controller specifics
  declare_parameter_if_not_declared(node, name_ + ".lookahead_dist", rclcpp::ParameterValue(0.8));
  node->get_parameter(name_ + ".lookahead_dist", lookahead_dist_);

  declare_parameter_if_not_declared(node, name_ + ".is_holonomic", rclcpp::ParameterValue(false));
  node->get_parameter(name_ + ".is_holonomic", is_holonomic_);

  declare_parameter_if_not_declared(node, name_ + ".kw", rclcpp::ParameterValue(1.5));
  node->get_parameter(name_ + ".kw", kw_);

  declare_parameter_if_not_declared(node, name_ + ".debug_mode", rclcpp::ParameterValue(false));
  node->get_parameter(name_ + ".debug_mode", debug_mode_);

  // Initialize Obstacle Clusterer
  obstacle_clusterer_ = std::make_shared<ObstacleClusterer>(parent, name_, tf_buffer_, costmap_ros_);

  marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("orca_markers", 1);
  diag_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 1);

  RCLCPP_INFO(logger_, "Configured OrcaController: %s (Holonomic: %d)", name_.c_str(), is_holonomic_);
}

void OrcaController::cleanup()
{
  obstacle_clusterer_.reset();
  sim_.reset();
  RCLCPP_INFO(logger_, "Cleaned up OrcaController: %s", name_.c_str());
}

void OrcaController::activate()
{
  marker_pub_->on_activate();
  diag_pub_->on_activate();
  RCLCPP_INFO(logger_, "Activated OrcaController: %s", name_.c_str());
}

void OrcaController::deactivate()
{
  marker_pub_->on_deactivate();
  diag_pub_->on_deactivate();
  RCLCPP_INFO(logger_, "Deactivated OrcaController: %s", name_.c_str());
}

void OrcaController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

void OrcaController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    speed_limit_ = max_speed_ * speed_limit / 100.0;
  } else {
    speed_limit_ = speed_limit;
  }
}

geometry_msgs::msg::PoseStamped OrcaController::getLookaheadPoint(
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  // Simple lookahead search
  double min_dist = std::numeric_limits<double>::max();
  size_t closest_idx = 0;
  
  for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
    double dx = global_plan_.poses[i].pose.position.x - robot_pose.pose.position.x;
    double dy = global_plan_.poses[i].pose.position.y - robot_pose.pose.position.y;
    double dist = std::hypot(dx, dy);
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }

  size_t lookahead_idx = closest_idx;
  for (size_t i = closest_idx; i < global_plan_.poses.size(); ++i) {
    double dx = global_plan_.poses[i].pose.position.x - robot_pose.pose.position.x;
    double dy = global_plan_.poses[i].pose.position.y - robot_pose.pose.position.y;
    double dist = std::hypot(dx, dy);
    if (dist >= lookahead_dist_) {
      lookahead_idx = i;
      break;
    }
    lookahead_idx = i;
  }

  return global_plan_.poses[lookahead_idx];
}

geometry_msgs::msg::TwistStamped OrcaController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  auto cm_start = std::chrono::high_resolution_clock::now();
  // Refresh obstacles from costmap
  obstacle_clusterer_->processCostmap();
  auto cm_end = std::chrono::high_resolution_clock::now();
  double costmap_time = std::chrono::duration<double, std::milli>(cm_end - cm_start).count();

  auto rvo_start = std::chrono::high_resolution_clock::now();
  // Create or reset RVO simulator for the step
  sim_ = std::make_unique<RVO::RVOSimulator>();
  sim_->setTimeStep(time_step_);
  sim_->setAgentDefaults(neighbor_dist_, max_neighbors_, time_horizon_, time_horizon_obst_, 
                         robot_radius_ + safety_buffer_, max_speed_);

  // Add Robot (Agent 0)
  size_t robot_id = sim_->addAgent(RVO::Vector2(robot_pose.pose.position.x, robot_pose.pose.position.y));
  
  // RVO2 uses cartesian velocities in the global frame
  double yaw = tf2::getYaw(robot_pose.pose.orientation);
  double vx_global = robot_speed.linear.x * std::cos(yaw) - robot_speed.linear.y * std::sin(yaw);
  double vy_global = robot_speed.linear.x * std::sin(yaw) + robot_speed.linear.y * std::cos(yaw);
  sim_->setAgentVelocity(robot_id, RVO::Vector2(vx_global, vy_global));

  // Determine Goal Velocity
  geometry_msgs::msg::PoseStamped lookahead = getLookaheadPoint(robot_pose);
  double gx = lookahead.pose.position.x - robot_pose.pose.position.x;
  double gy = lookahead.pose.position.y - robot_pose.pose.position.y;
  double dist_to_goal = std::hypot(gx, gy);

  RVO::Vector2 pref_vel(0.0, 0.0);
  if (dist_to_goal > 0.01) {
    double speed = std::min(speed_limit_, dist_to_goal / time_step_);
    pref_vel = RVO::Vector2(gx / dist_to_goal * speed, gy / dist_to_goal * speed);
  }
  sim_->setAgentPrefVelocity(robot_id, pref_vel);

  // Add Obstacles
  auto obstacles = obstacle_clusterer_->getObstacles();
  for (const auto & obs : obstacles) {
    if (obs.is_dynamic) {
      // Dynamic obstacles treated as other agents
      size_t id = sim_->addAgent(RVO::Vector2(obs.x, obs.y));
      sim_->setAgentRadius(id, obs.radius + safety_buffer_);
      sim_->setAgentVelocity(id, RVO::Vector2(obs.vx, obs.vy));
      sim_->setAgentPrefVelocity(id, RVO::Vector2(obs.vx, obs.vy));
    } else {
      // Static obstacles as non-moving agents (more efficient for circular representations)
      size_t id = sim_->addAgent(RVO::Vector2(obs.x, obs.y));
      sim_->setAgentRadius(id, obs.radius + safety_buffer_);
      sim_->setAgentVelocity(id, RVO::Vector2(0.0, 0.0));
      sim_->setAgentPrefVelocity(id, RVO::Vector2(0.0, 0.0));
      sim_->setAgentMaxSpeed(id, 0.0);
    }
  }

  // Simulate one step
  sim_->doStep();

  // Retrieve Safe Velocity in Global Frame
  RVO::Vector2 safe_vel = sim_->getAgentVelocity(robot_id);
  double v_x = safe_vel.x();
  double v_y = safe_vel.y();

  // Convert to Robot Frame
  double v_x_robot = v_x * std::cos(yaw) + v_y * std::sin(yaw);
  double v_y_robot = -v_x * std::sin(yaw) + v_y * std::cos(yaw);

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = robot_pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();

  if (is_holonomic_) {
    cmd_vel.twist.linear.x = v_x_robot;
    cmd_vel.twist.linear.y = v_y_robot;
    // Align orientation towards the target heading
    double target_yaw = std::atan2(gy, gx);
    double yaw_err = target_yaw - yaw;
    // normalize angle
    while (yaw_err > M_PI) yaw_err -= 2.0 * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;
    cmd_vel.twist.angular.z = kw_ * yaw_err;
  } else {
    // Differential Drive Kinematics Mapping
    double heading_err = std::atan2(v_y_robot, v_x_robot);
    double linear_speed = std::hypot(v_x_robot, v_y_robot);

    // If target is behind us, ORCA wants to move backwards or turn around
    if (std::cos(heading_err) < 0) {
       linear_speed = -linear_speed; 
       heading_err = std::atan2(-v_y_robot, -v_x_robot);
    }

    cmd_vel.twist.linear.x = linear_speed;
    cmd_vel.twist.linear.y = 0.0;
    cmd_vel.twist.angular.z = kw_ * heading_err;
  }

  auto rvo_end = std::chrono::high_resolution_clock::now();
  double rvo_time = std::chrono::duration<double, std::milli>(rvo_end - rvo_start).count();

  if (debug_mode_) {
    publishMarkers(robot_pose, cmd_vel, obstacles);
  }
  publishDiagnostics(obstacle_clusterer_->getLastLaserProcessTime(), costmap_time, rvo_time);

  return cmd_vel;
}

void OrcaController::publishDiagnostics(double laser_time, double costmap_time, double compute_time)
{
  diagnostic_msgs::msg::DiagnosticArray diag_array;
  diag_array.header.stamp = clock_->now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = name_ + ": Compute Times";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = "Controller timing metrics";

  diagnostic_msgs::msg::KeyValue kv_laser;
  kv_laser.key = "laser_process_time_ms";
  kv_laser.value = std::to_string(laser_time);
  status.values.push_back(kv_laser);

  diagnostic_msgs::msg::KeyValue kv_costmap;
  kv_costmap.key = "costmap_process_time_ms";
  kv_costmap.value = std::to_string(costmap_time);
  status.values.push_back(kv_costmap);

  diagnostic_msgs::msg::KeyValue kv_rvo;
  kv_rvo.key = "rvo2_compute_time_ms";
  kv_rvo.value = std::to_string(compute_time);
  status.values.push_back(kv_rvo);

  diagnostic_msgs::msg::KeyValue kv_total;
  kv_total.key = "total_cycle_time_ms";
  kv_total.value = std::to_string(laser_time + costmap_time + compute_time);
  status.values.push_back(kv_total);

  diag_array.status.push_back(status);
  diag_pub_->publish(diag_array);
}

void OrcaController::publishMarkers(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::TwistStamped & cmd_vel,
  const std::vector<OrcaObstacle> & obstacles)
{
  visualization_msgs::msg::MarkerArray marker_array;
  auto stamp = clock_->now();

  // Clear previous frame markers so removed obstacles do not persist
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.header.frame_id = costmap_ros_->getGlobalFrameID();
  delete_marker.header.stamp = stamp;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_marker);

  // 1. Obstacles currently considered by ORCA as Cylinders (Circles in 2D)
  visualization_msgs::msg::Marker obs_marker;
  obs_marker.header.frame_id = costmap_ros_->getGlobalFrameID();
  obs_marker.header.stamp = stamp;
  obs_marker.ns = "obstacles";
  obs_marker.type = visualization_msgs::msg::Marker::CYLINDER;
  obs_marker.action = visualization_msgs::msg::Marker::ADD;
  obs_marker.lifetime = rclcpp::Duration::from_seconds(0.5);

  for (size_t i = 0; i < obstacles.size(); ++i) {
    const auto & obs = obstacles[i];
    obs_marker.id = static_cast<int>(i);
    obs_marker.pose.position.x = obs.x;
    obs_marker.pose.position.y = obs.y;
    obs_marker.pose.position.z = 0.05;
    obs_marker.pose.orientation.w = 1.0;

    // Scale is diameter
    obs_marker.scale.x = obs.radius * 2.0;
    obs_marker.scale.y = obs.radius * 2.0;
    obs_marker.scale.z = 0.1;

    if (obs.is_dynamic) {
      obs_marker.color.r = 1.0;
      obs_marker.color.g = 0.5;
      obs_marker.color.b = 0.0;
      obs_marker.color.a = 0.7;
    } else {
      obs_marker.color.r = 1.0;
      obs_marker.color.g = 0.0;
      obs_marker.color.b = 0.0;
      obs_marker.color.a = 0.5;
    }
    marker_array.markers.push_back(obs_marker);
  }

  // 2. Predicted Trajectory Arrow (arc length proportional to linear velocity,
  // laterally deformed by angular velocity)
  const double vx = cmd_vel.twist.linear.x;
  const double vy = cmd_vel.twist.linear.y;
  const double wz = cmd_vel.twist.angular.z;
  const double linear_speed = std::hypot(vx, vy);

  if (linear_speed > 1e-3) {
    const double horizon = 1.0;  // 1s rollout: total arc length equals linear_speed * 1.0 m
    const int num_steps = 20;
    const double dt = horizon / num_steps;

    double x = robot_pose.pose.position.x;
    double y = robot_pose.pose.position.y;
    double theta = tf2::getYaw(robot_pose.pose.orientation);

    visualization_msgs::msg::Marker shaft_marker;
    shaft_marker.header.frame_id = robot_pose.header.frame_id;
    shaft_marker.header.stamp = stamp;
    shaft_marker.ns = "trajectory_shaft";
    shaft_marker.id = 0;
    shaft_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    shaft_marker.action = visualization_msgs::msg::Marker::ADD;
    shaft_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    shaft_marker.scale.x = 0.04;
    shaft_marker.color.r = 0.0;
    shaft_marker.color.g = 1.0;
    shaft_marker.color.b = 0.0;
    shaft_marker.color.a = 1.0;

    geometry_msgs::msg::Point pt;
    pt.z = 0.1;
    for (int k = 0; k <= num_steps; ++k) {
      pt.x = x;
      pt.y = y;
      shaft_marker.points.push_back(pt);

      x += (vx * std::cos(theta) - vy * std::sin(theta)) * dt;
      y += (vx * std::sin(theta) + vy * std::cos(theta)) * dt;
      theta += wz * dt;
    }
    marker_array.markers.push_back(shaft_marker);

    // Arrowhead at the end of the curved trajectory aligned with the final segment
    const auto & p_prev = shaft_marker.points[shaft_marker.points.size() - 2];
    const auto & p_tip = shaft_marker.points.back();
    double seg_dx = p_tip.x - p_prev.x;
    double seg_dy = p_tip.y - p_prev.y;
    double seg_len = std::hypot(seg_dx, seg_dy);

    if (seg_len > 1e-6) {
      visualization_msgs::msg::Marker head_marker;
      head_marker.header.frame_id = robot_pose.header.frame_id;
      head_marker.header.stamp = stamp;
      head_marker.ns = "trajectory_head";
      head_marker.id = 1;
      head_marker.type = visualization_msgs::msg::Marker::ARROW;
      head_marker.action = visualization_msgs::msg::Marker::ADD;
      head_marker.lifetime = rclcpp::Duration::from_seconds(0.5);

      double head_len = std::min(0.12, linear_speed * horizon * 0.4);
      geometry_msgs::msg::Point p_tail;
      p_tail.x = p_tip.x - (seg_dx / seg_len) * head_len;
      p_tail.y = p_tip.y - (seg_dy / seg_len) * head_len;
      p_tail.z = 0.1;

      head_marker.points.push_back(p_tail);
      head_marker.points.push_back(p_tip);

      head_marker.scale.x = 0.04;
      head_marker.scale.y = 0.10;
      head_marker.scale.z = head_len;

      head_marker.color.r = 0.0;
      head_marker.color.g = 1.0;
      head_marker.color.b = 0.0;
      head_marker.color.a = 1.0;

      marker_array.markers.push_back(head_marker);
    }
  }

  marker_pub_->publish(marker_array);
}

}  // namespace nav2_orca_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_orca_controller::OrcaController, nav2_core::Controller)

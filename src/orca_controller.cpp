#include "nav2_orca_controller/orca_controller.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include <chrono>

using nav2_util::declare_parameter_if_not_declared;

namespace nav2_orca_controller
{

namespace
{

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

}  // namespace

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
  declare_parameter_if_not_declared(node, name_ + ".lookahead_dist", rclcpp::ParameterValue(0.45));
  node->get_parameter(name_ + ".lookahead_dist", lookahead_dist_);

  declare_parameter_if_not_declared(node, name_ + ".is_holonomic", rclcpp::ParameterValue(false));
  node->get_parameter(name_ + ".is_holonomic", is_holonomic_);

  declare_parameter_if_not_declared(node, name_ + ".kw", rclcpp::ParameterValue(1.8));
  node->get_parameter(name_ + ".kw", kw_);

  declare_parameter_if_not_declared(node, name_ + ".max_angular_vel", rclcpp::ParameterValue(0.8));
  node->get_parameter(name_ + ".max_angular_vel", max_angular_vel_);

  declare_parameter_if_not_declared(node, name_ + ".debug_mode", rclcpp::ParameterValue(false));
  node->get_parameter(name_ + ".debug_mode", debug_mode_);

  // Goal convergence parameters
  declare_parameter_if_not_declared(
    node, name_ + ".rotate_to_goal_heading", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + ".rotate_to_goal_heading", rotate_to_goal_heading_);

  declare_parameter_if_not_declared(
    node, name_ + ".goal_dist_tolerance", rclcpp::ParameterValue(0.20));
  node->get_parameter(name_ + ".goal_dist_tolerance", goal_dist_tolerance_);

  declare_parameter_if_not_declared(
    node, name_ + ".goal_yaw_tolerance", rclcpp::ParameterValue(0.15));
  node->get_parameter(name_ + ".goal_yaw_tolerance", goal_yaw_tolerance_);

  declare_parameter_if_not_declared(
    node, name_ + ".goal_decel_distance", rclcpp::ParameterValue(0.50));
  node->get_parameter(name_ + ".goal_decel_distance", goal_decel_distance_);

  declare_parameter_if_not_declared(
    node, name_ + ".min_approach_speed", rclcpp::ParameterValue(0.06));
  node->get_parameter(name_ + ".min_approach_speed", min_approach_speed_);

  // Forward preference & short reversal parameters
  declare_parameter_if_not_declared(
    node, name_ + ".allow_short_reversal", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + ".allow_short_reversal", allow_short_reversal_);

  declare_parameter_if_not_declared(
    node, name_ + ".min_backward_angle", rclcpp::ParameterValue(2.44));
  node->get_parameter(name_ + ".min_backward_angle", min_backward_angle_);

  declare_parameter_if_not_declared(
    node, name_ + ".max_backward_distance", rclcpp::ParameterValue(0.35));
  node->get_parameter(name_ + ".max_backward_distance", max_backward_distance_);

  declare_parameter_if_not_declared(
    node, name_ + ".max_backward_duration", rclcpp::ParameterValue(1.5));
  node->get_parameter(name_ + ".max_backward_duration", max_backward_duration_);

  declare_parameter_if_not_declared(
    node, name_ + ".max_backward_speed", rclcpp::ParameterValue(0.15));
  node->get_parameter(name_ + ".max_backward_speed", max_backward_speed_);

  declare_parameter_if_not_declared(
    node, name_ + ".forward_turn_slowdown_angle", rclcpp::ParameterValue(1.22));
  node->get_parameter(name_ + ".forward_turn_slowdown_angle", forward_turn_slowdown_angle_);

  // Initialize Obstacle Clusterer
  obstacle_clusterer_ = std::make_shared<ObstacleClusterer>(parent, name_, tf_buffer_, costmap_ros_);

  marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("orca_markers", 1);
  diag_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 1);

  RCLCPP_INFO(
    logger_, "Configured OrcaController: %s (Holonomic: %d, AllowShortReversal: %d)",
    name_.c_str(), is_holonomic_, allow_short_reversal_);
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
  is_reversing_ = false;
  reverse_lockout_ = false;
  in_place_turning_ = false;
  xy_goal_reached_ = false;
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
  // If goal moved significantly compared to previous plan, reset xy_goal_reached_
  if (!global_plan_.poses.empty() && !path.poses.empty()) {
    const auto & old_goal = global_plan_.poses.back().pose.position;
    const auto & new_goal = path.poses.back().pose.position;
    if (std::hypot(new_goal.x - old_goal.x, new_goal.y - old_goal.y) > goal_dist_tolerance_) {
      xy_goal_reached_ = false;
    }
  } else {
    xy_goal_reached_ = false;
  }

  global_plan_ = path;
  closest_plan_idx_ = 0;
}

void OrcaController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    speed_limit_ = max_speed_ * speed_limit / 100.0;
  } else {
    speed_limit_ = speed_limit;
  }
}

nav_msgs::msg::Path OrcaController::transformPlanToFrame(
  const nav_msgs::msg::Path & plan,
  const std::string & target_frame,
  const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
  const rclcpp::Logger & logger)
{
  if (plan.poses.empty() || target_frame.empty()) {
    return plan;
  }

  const std::string source_frame =
    !plan.header.frame_id.empty() ? plan.header.frame_id : plan.poses.front().header.frame_id;

  if (source_frame.empty() || source_frame == target_frame || !tf_buffer) {
    return plan;
  }

  geometry_msgs::msg::TransformStamped tf_stamped;
  try {
    tf_stamped = tf_buffer->lookupTransform(
      target_frame, source_frame, tf2::TimePointZero, tf2::durationFromSec(0.2));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      logger, "Failed to transform global plan from %s to %s: %s",
      source_frame.c_str(), target_frame.c_str(), ex.what());
    return plan;
  }

  nav_msgs::msg::Path transformed;
  transformed.header.frame_id = target_frame;
  transformed.header.stamp = plan.header.stamp;
  transformed.poses.resize(plan.poses.size());

  for (size_t i = 0; i < plan.poses.size(); ++i) {
    tf2::doTransform(plan.poses[i], transformed.poses[i], tf_stamped);
    transformed.poses[i].header.frame_id = target_frame;
  }

  return transformed;
}

geometry_msgs::msg::PoseStamped OrcaController::getLookaheadPointFromPlan(
  const nav_msgs::msg::Path & transformed_plan,
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  if (transformed_plan.poses.empty()) {
    return robot_pose;
  }

  if (closest_plan_idx_ >= transformed_plan.poses.size()) {
    closest_plan_idx_ = 0;
  }

  // Monotonic forward pruning: search from closest_plan_idx_ within a local window along the path
  // so that already traversed segments behind the robot are never re-selected.
  double min_dist = std::numeric_limits<double>::max();
  size_t best_idx = closest_plan_idx_;
  double acc_path_dist = 0.0;
  const double max_search_window = std::max(lookahead_dist_ * 3.0, 2.0);

  for (size_t i = closest_plan_idx_; i < transformed_plan.poses.size(); ++i) {
    if (i > closest_plan_idx_) {
      double seg_dx =
        transformed_plan.poses[i].pose.position.x - transformed_plan.poses[i - 1].pose.position.x;
      double seg_dy =
        transformed_plan.poses[i].pose.position.y - transformed_plan.poses[i - 1].pose.position.y;
      acc_path_dist += std::hypot(seg_dx, seg_dy);
    }

    double dx = transformed_plan.poses[i].pose.position.x - robot_pose.pose.position.x;
    double dy = transformed_plan.poses[i].pose.position.y - robot_pose.pose.position.y;
    double dist = std::hypot(dx, dy);
    if (dist < min_dist) {
      min_dist = dist;
      best_idx = i;
    }

    if (acc_path_dist > max_search_window && dist > min_dist + lookahead_dist_) {
      break;
    }
  }

  closest_plan_idx_ = best_idx;

  // Interpolate lookahead point along the path segment for smooth tracking without discretization jumps
  for (size_t i = closest_plan_idx_; i < transformed_plan.poses.size(); ++i) {
    double dx = transformed_plan.poses[i].pose.position.x - robot_pose.pose.position.x;
    double dy = transformed_plan.poses[i].pose.position.y - robot_pose.pose.position.y;
    double dist = std::hypot(dx, dy);

    if (dist >= lookahead_dist_) {
      if (i > closest_plan_idx_) {
        const auto & p_prev = transformed_plan.poses[i - 1].pose.position;
        const auto & p_curr = transformed_plan.poses[i].pose.position;
        const double prev_dist = std::hypot(
          p_prev.x - robot_pose.pose.position.x,
          p_prev.y - robot_pose.pose.position.y);
        const double denom = dist - prev_dist;
        if (denom > 1e-4 && lookahead_dist_ >= prev_dist) {
          const double alpha = std::clamp((lookahead_dist_ - prev_dist) / denom, 0.0, 1.0);
          geometry_msgs::msg::PoseStamped interp = transformed_plan.poses[i];
          interp.pose.position.x = p_prev.x + alpha * (p_curr.x - p_prev.x);
          interp.pose.position.y = p_prev.y + alpha * (p_curr.y - p_prev.y);
          return interp;
        }
      }
      return transformed_plan.poses[i];
    }
  }

  return transformed_plan.poses.back();
}

geometry_msgs::msg::PoseStamped OrcaController::getLookaheadPoint(
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  auto transformed_plan = transformPlanToFrame(
    global_plan_, robot_pose.header.frame_id, tf_buffer_, logger_);
  return getLookaheadPointFromPlan(transformed_plan, robot_pose);
}

bool OrcaController::computeGoalArrivalCommand(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::PoseStamped & final_goal_pose,
  double xy_tolerance,
  double yaw_tolerance,
  geometry_msgs::msg::Twist & out_twist)
{
  const double dx_goal = final_goal_pose.pose.position.x - robot_pose.pose.position.x;
  const double dy_goal = final_goal_pose.pose.position.y - robot_pose.pose.position.y;
  const double dist_to_final_goal = std::hypot(dx_goal, dy_goal);

  // Hysteresis on XY goal latch so the robot does not toggle between translation and final rotation
  if (dist_to_final_goal <= xy_tolerance) {
    xy_goal_reached_ = true;
  } else if (dist_to_final_goal > xy_tolerance * 1.35) {
    xy_goal_reached_ = false;
  }

  if (!xy_goal_reached_) {
    return false;
  }

  // Within XY goal tolerance: stop linear motion and align with final goal yaw
  out_twist.linear.x = 0.0;
  out_twist.linear.y = 0.0;
  is_reversing_ = false;
  in_place_turning_ = false;

  if (!rotate_to_goal_heading_) {
    out_twist.angular.z = 0.0;
    return true;
  }

  const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  const double goal_yaw = tf2::getYaw(final_goal_pose.pose.orientation);
  const double yaw_err = normalizeAngle(goal_yaw - robot_yaw);

  if (std::abs(yaw_err) <= yaw_tolerance * 0.5) {
    out_twist.angular.z = 0.0;
  } else {
    double wz = kw_ * yaw_err;
    // Apply a small minimum rotational speed when outside tolerance to overcome friction/deadband
    const double min_rot_vel = 0.10;
    if (std::abs(wz) < min_rot_vel) {
      wz = std::copysign(min_rot_vel, yaw_err);
    }
    out_twist.angular.z = std::clamp(wz, -max_angular_vel_, max_angular_vel_);
  }

  return true;
}

void OrcaController::mapDiffDriveVelocity(
  double v_x_robot,
  double v_y_robot,
  const geometry_msgs::msg::Point & current_pos,
  const rclcpp::Time & now,
  geometry_msgs::msg::Twist & out_twist)
{
  out_twist.linear.y = 0.0;
  const double desired_speed = std::hypot(v_x_robot, v_y_robot);

  if (desired_speed < 1e-3) {
    out_twist.linear.x = 0.0;
    out_twist.angular.z = 0.0;
    is_reversing_ = false;
    in_place_turning_ = false;
    return;
  }

  const double heading_err = std::atan2(v_y_robot, v_x_robot);
  const double abs_heading_err = std::abs(heading_err);

  // Release reverse lockout once the desired velocity points forward again
  if (abs_heading_err < M_PI_2) {
    is_reversing_ = false;
    reverse_lockout_ = false;
  }

  // Check if short backward maneuver is eligible
  bool wants_reverse = false;
  if (allow_short_reversal_ && !reverse_lockout_) {
    if (!is_reversing_ && abs_heading_err >= min_backward_angle_) {
      is_reversing_ = true;
      reverse_start_time_ = now;
      reverse_start_pos_ = current_pos;
    }
    if (is_reversing_ && abs_heading_err >= M_PI_2) {
      const double rev_dist = std::hypot(
        current_pos.x - reverse_start_pos_.x,
        current_pos.y - reverse_start_pos_.y);
      const double rev_time = (now - reverse_start_time_).seconds();

      if (rev_dist <= max_backward_distance_ && rev_time <= max_backward_duration_) {
        wants_reverse = true;
      } else {
        // Reversal budget exhausted: lock out further backing up and force a forward turn
        is_reversing_ = false;
        reverse_lockout_ = true;
      }
    } else {
      is_reversing_ = false;
    }
  } else {
    is_reversing_ = false;
  }

  if (wants_reverse) {
    in_place_turning_ = false;
    const double rear_heading_err = std::atan2(-v_y_robot, -v_x_robot);
    out_twist.linear.x = -std::min(desired_speed, max_backward_speed_);
    out_twist.angular.z = std::clamp(kw_ * rear_heading_err, -max_angular_vel_, max_angular_vel_);
    return;
  }

  // Forward-preference kinematics with turn-in-place hysteresis:
  // Engage in-place rotation when abs_heading_err >= slowdown_angle (~70 deg),
  // and remain in in-place rotation until abs_heading_err <= disengage_angle (~25 deg)
  // so the robot aligns with the path instead of driving in a wide offset arc or chattering.
  const double slowdown_angle = std::max(0.2, forward_turn_slowdown_angle_);
  const double disengage_angle = std::min(0.45, slowdown_angle * 0.4);

  if (abs_heading_err >= slowdown_angle) {
    in_place_turning_ = true;
  } else if (abs_heading_err <= disengage_angle) {
    in_place_turning_ = false;
  }

  if (in_place_turning_) {
    out_twist.linear.x = 0.0;
    double wz = kw_ * heading_err;
    const double min_turn_vel = 0.15;
    if (std::abs(wz) < min_turn_vel) {
      wz = std::copysign(min_turn_vel, heading_err);
    }
    out_twist.angular.z = std::clamp(wz, -max_angular_vel_, max_angular_vel_);
  } else {
    const double angle_scale = std::cos(heading_err * (M_PI_2 / slowdown_angle));
    out_twist.linear.x = desired_speed * std::max(0.0, angle_scale);
    out_twist.angular.z = std::clamp(kw_ * heading_err, -max_angular_vel_, max_angular_vel_);
  }
}

geometry_msgs::msg::TwistStamped OrcaController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  nav2_core::GoalChecker * goal_checker)
{
  auto cm_start = std::chrono::high_resolution_clock::now();
  // Refresh static polygons and small obstacles from costmap
  obstacle_clusterer_->processCostmap();
  auto cm_end = std::chrono::high_resolution_clock::now();
  double costmap_time = std::chrono::duration<double, std::milli>(cm_end - cm_start).count();

  auto rvo_start = std::chrono::high_resolution_clock::now();
  auto now = clock_->now();

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = robot_pose.header.frame_id;
  cmd_vel.header.stamp = now;

  // Transform global_plan_ from its frame (e.g. "map") into robot_pose's frame (e.g. "odom")
  auto transformed_plan = transformPlanToFrame(
    global_plan_, robot_pose.header.frame_id, tf_buffer_, logger_);

  if (transformed_plan.poses.empty()) {
    return cmd_vel;
  }

  // Query tolerances from Nav2's GoalChecker when provided
  double xy_tol = goal_dist_tolerance_;
  double yaw_tol = goal_yaw_tolerance_;
  if (goal_checker != nullptr) {
    geometry_msgs::msg::Pose pose_tol;
    geometry_msgs::msg::Twist vel_tol;
    if (goal_checker->getTolerances(pose_tol, vel_tol)) {
      if (pose_tol.position.x > 1e-4) {
        xy_tol = pose_tol.position.x;
      }
      const double gc_yaw_tol = tf2::getYaw(pose_tol.orientation);
      if (std::abs(gc_yaw_tol) > 1e-4) {
        yaw_tol = std::abs(gc_yaw_tol);
      }
    }
  }

  const auto & final_goal_pose = transformed_plan.poses.back();
  auto polygons = obstacle_clusterer_->getPolygons();
  auto obstacles = obstacle_clusterer_->getObstacles();

  // Check if we have reached the XY goal region and only need final heading alignment
  if (computeGoalArrivalCommand(
      robot_pose, final_goal_pose, xy_tol, yaw_tol, cmd_vel.twist))
  {
    auto rvo_end = std::chrono::high_resolution_clock::now();
    double rvo_time = std::chrono::duration<double, std::milli>(rvo_end - rvo_start).count();
    if (debug_mode_) {
      publishMarkers(robot_pose, cmd_vel, obstacles, polygons);
    }
    publishDiagnostics(obstacle_clusterer_->getLastLaserProcessTime(), costmap_time, rvo_time);
    return cmd_vel;
  }

  // Create or reset RVO simulator for the step
  sim_ = std::make_unique<RVO::RVOSimulator>();
  sim_->setTimeStep(time_step_);
  sim_->setAgentDefaults(
    neighbor_dist_, max_neighbors_, time_horizon_, time_horizon_obst_,
    robot_radius_ + safety_buffer_, max_speed_);

  // Add Robot (Agent 0)
  size_t robot_id = sim_->addAgent(
    RVO::Vector2(robot_pose.pose.position.x, robot_pose.pose.position.y));

  // RVO2 uses cartesian velocities in the global frame
  double yaw = tf2::getYaw(robot_pose.pose.orientation);
  double vx_global = robot_speed.linear.x * std::cos(yaw) - robot_speed.linear.y * std::sin(yaw);
  double vy_global = robot_speed.linear.x * std::sin(yaw) + robot_speed.linear.y * std::cos(yaw);
  sim_->setAgentVelocity(robot_id, RVO::Vector2(vx_global, vy_global));

  // Determine Lookahead Point & Smooth Goal Approach Velocity
  geometry_msgs::msg::PoseStamped lookahead =
    getLookaheadPointFromPlan(transformed_plan, robot_pose);
  double gx = lookahead.pose.position.x - robot_pose.pose.position.x;
  double gy = lookahead.pose.position.y - robot_pose.pose.position.y;
  double dist_to_lookahead = std::hypot(gx, gy);

  const double dist_to_final_goal = std::hypot(
    final_goal_pose.pose.position.x - robot_pose.pose.position.x,
    final_goal_pose.pose.position.y - robot_pose.pose.position.y);

  RVO::Vector2 pref_vel(0.0, 0.0);
  if (dist_to_lookahead > 1e-3) {
    double target_speed = speed_limit_;
    if (goal_decel_distance_ > 1e-3 && dist_to_final_goal < goal_decel_distance_) {
      const double decel_ratio = std::clamp(dist_to_final_goal / goal_decel_distance_, 0.0, 1.0);
      target_speed = std::max(min_approach_speed_, speed_limit_ * decel_ratio);
    }
    target_speed = std::min(target_speed, dist_to_final_goal / std::max(0.05, time_step_));
    pref_vel = RVO::Vector2(
      gx / dist_to_lookahead * target_speed,
      gy / dist_to_lookahead * target_speed);
  }
  sim_->setAgentPrefVelocity(robot_id, pref_vel);

  // 1. Add Static Polygonal Obstacles from Costmap natively to RVO2
  for (const auto & poly : polygons) {
    if (poly.vertices.size() < 3) {
      continue;
    }
    std::vector<RVO::Vector2> rvo_vertices;
    rvo_vertices.reserve(poly.vertices.size());
    for (const auto & pt : poly.vertices) {
      rvo_vertices.emplace_back(static_cast<float>(pt.x), static_cast<float>(pt.y));
    }
    sim_->addObstacle(rvo_vertices);
  }
  if (!polygons.empty()) {
    sim_->processObstacles();
  }

  // 2. Add Circular Obstacles (LiDAR clusters, dynamic pedestrians, small blobs)
  for (const auto & obs : obstacles) {
    if (obs.is_dynamic) {
      size_t id = sim_->addAgent(RVO::Vector2(obs.x, obs.y));
      sim_->setAgentRadius(id, obs.radius + safety_buffer_);
      sim_->setAgentVelocity(id, RVO::Vector2(obs.vx, obs.vy));
      sim_->setAgentPrefVelocity(id, RVO::Vector2(obs.vx, obs.vy));
    } else {
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

  if (is_holonomic_) {
    cmd_vel.twist.linear.x = v_x_robot;
    cmd_vel.twist.linear.y = v_y_robot;
    double target_yaw = std::atan2(gy, gx);
    double yaw_err = normalizeAngle(target_yaw - yaw);
    cmd_vel.twist.angular.z = std::clamp(kw_ * yaw_err, -max_angular_vel_, max_angular_vel_);
  } else {
    mapDiffDriveVelocity(
      v_x_robot, v_y_robot, robot_pose.pose.position, now, cmd_vel.twist);
  }

  auto rvo_end = std::chrono::high_resolution_clock::now();
  double rvo_time = std::chrono::duration<double, std::milli>(rvo_end - rvo_start).count();

  if (debug_mode_) {
    publishMarkers(robot_pose, cmd_vel, obstacles, polygons);
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
  const std::vector<OrcaObstacle> & obstacles,
  const std::vector<OrcaPolygon> & polygons)
{
  visualization_msgs::msg::MarkerArray marker_array;
  auto stamp = clock_->now();

  // Clear previous frame markers so removed obstacles do not persist
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.header.frame_id = costmap_ros_->getGlobalFrameID();
  delete_marker.header.stamp = stamp;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_marker);

  // 1. Circular Obstacles currently considered by ORCA as Cylinders (Circles in 2D)
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

  // 1b. Static Polygonal Obstacles extracted from Costmap
  for (size_t i = 0; i < polygons.size(); ++i) {
    const auto & poly = polygons[i];
    if (poly.vertices.size() < 3) {
      continue;
    }
    visualization_msgs::msg::Marker poly_marker;
    poly_marker.header.frame_id = costmap_ros_->getGlobalFrameID();
    poly_marker.header.stamp = stamp;
    poly_marker.ns = "obstacle_polygons";
    poly_marker.id = static_cast<int>(i);
    poly_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    poly_marker.action = visualization_msgs::msg::Marker::ADD;
    poly_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    poly_marker.pose.orientation.w = 1.0;
    poly_marker.scale.x = 0.03;

    poly_marker.color.r = 1.0;
    poly_marker.color.g = 0.15;
    poly_marker.color.b = 0.15;
    poly_marker.color.a = 0.9;

    for (const auto & v : poly.vertices) {
      geometry_msgs::msg::Point p = v;
      p.z = 0.08;
      poly_marker.points.push_back(p);
    }
    // Close the polygon loop
    geometry_msgs::msg::Point p_first = poly.vertices.front();
    p_first.z = 0.08;
    poly_marker.points.push_back(p_first);

    marker_array.markers.push_back(poly_marker);
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

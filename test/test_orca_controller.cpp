#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <vector>

#include "nav2_orca_controller/obstacle_clusterer.hpp"
#include "nav2_orca_controller/orca_controller.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using nav2_orca_controller::ObstacleClusterer;
using nav2_orca_controller::OrcaController;
using nav2_orca_controller::OrcaObstacle;
using nav2_orca_controller::OrcaPolygon;

TEST(ObstacleClustererTest, SubdivideLargeClusterCoversAllPointsWithBoundedRadius)
{
  // Create a 3.0m long wall cluster sampled every 0.05m
  std::vector<geometry_msgs::msg::Point> wall_cluster;
  for (double x = 0.0; x <= 3.0; x += 0.05) {
    geometry_msgs::msg::Point pt;
    pt.x = x;
    pt.y = 1.0;
    pt.z = 0.0;
    wall_cluster.push_back(pt);
  }

  const double max_radius = 0.25;
  const double padding = 0.05;
  std::vector<OrcaObstacle> circles;
  ObstacleClusterer::subdivideCluster(wall_cluster, max_radius, padding, circles);

  // Should generate multiple circles instead of a single truncated circle
  EXPECT_GE(circles.size(), 8u);

  for (const auto & c : circles) {
    EXPECT_LE(c.radius, max_radius + 1e-6);
    EXPECT_GT(c.radius, 0.0);
  }

  // Every original wall point must be covered by at least one circle
  for (const auto & pt : wall_cluster) {
    bool covered = false;
    for (const auto & c : circles) {
      double dist = std::hypot(pt.x - c.x, pt.y - c.y);
      if (dist <= c.radius + 1e-4) {
        covered = true;
        break;
      }
    }
    EXPECT_TRUE(covered) << "Point (" << pt.x << ", " << pt.y << ") was not covered by any circle!";
  }
}

TEST(ObstacleClustererTest, Subdivide2x4RectangleFillsWithMultipleCircles)
{
  // Create a 2m x 4m rectangular perimeter/surface point cloud
  std::vector<geometry_msgs::msg::Point> rect_points;
  for (double x = 0.0; x <= 4.0; x += 0.2) {
    for (double y = 0.0; y <= 2.0; y += 0.2) {
      geometry_msgs::msg::Point pt;
      pt.x = x;
      pt.y = y;
      pt.z = 0.0;
      rect_points.push_back(pt);
    }
  }

  // With max_radius = 1.05m (2.1m diameter), a 2x4m rectangle should split into multiple circles
  std::vector<OrcaObstacle> circles;
  ObstacleClusterer::subdivideCluster(rect_points, 1.05, 0.05, circles);
  EXPECT_GE(circles.size(), 2u);
  for (const auto & c : circles) {
    EXPECT_LE(c.radius, 1.05 + 1e-6);
  }
}

TEST(ObstacleClustererTest, ExtractPolygonsFromCostmapProducesCCWSimplifiedRectangle)
{
  // 40 x 40 cells at 0.05m resolution (2m x 2m map)
  nav2_costmap_2d::Costmap2D costmap(40, 40, 0.05, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);

  // Fill a rectangular wall from x=[10..29], y=[15..18] (20x4 = 80 lethal cells)
  for (unsigned int y = 15; y <= 18; ++y) {
    for (unsigned int x = 10; x <= 29; ++x) {
      costmap.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }

  // Add a single isolated lethal cell at (5, 5) (< min_polygon_cells)
  costmap.setCost(5, 5, nav2_costmap_2d::LETHAL_OBSTACLE);

  std::vector<OrcaPolygon> polygons;
  std::vector<OrcaObstacle> small_obstacles;
  ObstacleClusterer::extractFromCostmap(
    &costmap,
    /*min_polygon_cells=*/3,
    /*max_obstacle_radius=*/0.25,
    /*simplification_tolerance=*/0.05,
    polygons,
    small_obstacles);

  ASSERT_EQ(polygons.size(), 1u);
  ASSERT_EQ(small_obstacles.size(), 1u);

  // A straight axis-aligned rectangle should simplify to 4 corner vertices in CCW order
  EXPECT_EQ(polygons[0].vertices.size(), 4u);
  double signed_area = ObstacleClusterer::computeSignedArea(polygons[0].vertices);
  // 20 cells * 0.05 = 1.0m wide; 4 cells * 0.05 = 0.20m tall -> area = 0.20 m^2 > 0 (CCW)
  EXPECT_NEAR(signed_area, 0.20, 1e-3);
}

TEST(OrcaControllerKinematicsTest, ForwardPreferenceAvoidsReversingOnModerateBackwardAngle)
{
  OrcaController controller;
  geometry_msgs::msg::Point pos;
  pos.x = 0.0;
  pos.y = 0.0;
  rclcpp::Time t0(10, 0, RCL_ROS_TIME);

  // Desired velocity at 110 degrees (behind the 90 deg threshold, but < 140 deg min_backward_angle)
  const double angle_110 = 110.0 * M_PI / 180.0;
  const double vx = 0.3 * std::cos(angle_110);
  const double vy = 0.3 * std::sin(angle_110);

  geometry_msgs::msg::Twist cmd;
  controller.mapDiffDriveVelocity(vx, vy, pos, t0, cmd);

  // Must NOT reverse; should stop linear motion and turn forward in-place (clamped to max_angular_vel)
  EXPECT_DOUBLE_EQ(cmd.linear.x, 0.0);
  EXPECT_GT(cmd.angular.z, 0.0);
  EXPECT_LE(cmd.angular.z, 0.8 + 1e-6);
}

TEST(OrcaControllerKinematicsTest, ShortReversalAllowedThenLockedOutAfterDistanceOrTimeLimit)
{
  OrcaController controller;
  geometry_msgs::msg::Point pos;
  pos.x = 0.0;
  pos.y = 0.0;
  rclcpp::Time t0(10, 0, RCL_ROS_TIME);

  // Desired velocity at 170 degrees (>= min_backward_angle of ~140 deg)
  const double angle_170 = 170.0 * M_PI / 180.0;
  const double vx = 0.4 * std::cos(angle_170);
  const double vy = 0.4 * std::sin(angle_170);

  geometry_msgs::msg::Twist cmd;
  controller.mapDiffDriveVelocity(vx, vy, pos, t0, cmd);

  // Initially allowed to reverse at capped backward speed (<= 0.15 m/s)
  EXPECT_LT(cmd.linear.x, 0.0);
  EXPECT_GE(cmd.linear.x, -0.15 - 1e-6);

  // Move robot 0.40m backwards (exceeds max_backward_distance_ = 0.35m)
  pos.x = -0.40;
  rclcpp::Time t1(11, 0, RCL_ROS_TIME);
  controller.mapDiffDriveVelocity(vx, vy, pos, t1, cmd);

  // Reversal budget exhausted -> must stop reversing and rotate in place to align front
  EXPECT_DOUBLE_EQ(cmd.linear.x, 0.0);
  EXPECT_GT(std::abs(cmd.angular.z), 0.1);
  EXPECT_LE(std::abs(cmd.angular.z), 0.8 + 1e-6);
}

TEST(OrcaControllerFrameAndGoalTest, TransformPlanCorrectsMapToOdomOffset)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(clock);

  // Set transform from "map" to "odom" with offset (+0.5m, -0.35m)
  geometry_msgs::msg::TransformStamped tf_odom_from_map;
  tf_odom_from_map.header.stamp = rclcpp::Time(1, 0, RCL_ROS_TIME);
  tf_odom_from_map.header.frame_id = "odom";
  tf_odom_from_map.child_frame_id = "map";
  tf_odom_from_map.transform.translation.x = 0.50;
  tf_odom_from_map.transform.translation.y = -0.35;
  tf_odom_from_map.transform.translation.z = 0.0;
  tf_odom_from_map.transform.rotation.w = 1.0;
  tf_buffer->setTransform(tf_odom_from_map, "unit_test", true);

  nav_msgs::msg::Path plan_in_map;
  plan_in_map.header.frame_id = "map";
  for (int i = 0; i <= 10; ++i) {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "map";
    p.pose.position.x = i * 0.1;
    p.pose.position.y = 1.0;
    p.pose.orientation.w = 1.0;
    plan_in_map.poses.push_back(p);
  }

  auto plan_in_odom = OrcaController::transformPlanToFrame(
    plan_in_map, "odom", tf_buffer, rclcpp::get_logger("test"));

  ASSERT_EQ(plan_in_odom.poses.size(), plan_in_map.poses.size());
  EXPECT_EQ(plan_in_odom.header.frame_id, "odom");
  EXPECT_NEAR(plan_in_odom.poses.front().pose.position.x, 0.50, 1e-6);
  EXPECT_NEAR(plan_in_odom.poses.front().pose.position.y, 0.65, 1e-6);
  EXPECT_NEAR(plan_in_odom.poses.back().pose.position.x, 1.50, 1e-6);
  EXPECT_NEAR(plan_in_odom.poses.back().pose.position.y, 0.65, 1e-6);
}

TEST(OrcaControllerFrameAndGoalTest, GoalConvergenceStopsTranslationAndRotatesToGoalYaw)
{
  OrcaController controller;

  geometry_msgs::msg::PoseStamped final_goal;
  final_goal.header.frame_id = "odom";
  final_goal.pose.position.x = 2.0;
  final_goal.pose.position.y = 3.0;
  tf2::Quaternion q_goal;
  q_goal.setRPY(0.0, 0.0, M_PI_2);  // Goal yaw = +90 deg
  final_goal.pose.orientation = tf2::toMsg(q_goal);

  // Robot is 0.12m from final goal (inside xy_tolerance = 0.20m), facing 0 deg
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = "odom";
  robot_pose.pose.position.x = 1.90;
  robot_pose.pose.position.y = 2.94;
  robot_pose.pose.orientation.w = 1.0;

  geometry_msgs::msg::Twist cmd;
  bool arrived_xy = controller.computeGoalArrivalCommand(
    robot_pose, final_goal, /*xy_tolerance=*/0.20, /*yaw_tolerance=*/0.17, cmd);

  EXPECT_TRUE(arrived_xy);
  EXPECT_DOUBLE_EQ(cmd.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(cmd.linear.y, 0.0);
  // Should rotate positively towards +90 deg, clamped by max_angular_vel (0.8 rad/s)
  EXPECT_GT(cmd.angular.z, 0.1);
  EXPECT_LE(cmd.angular.z, 0.8 + 1e-6);

  // Once aligned with final goal yaw, angular velocity should drop to 0
  robot_pose.pose.orientation = tf2::toMsg(q_goal);
  arrived_xy = controller.computeGoalArrivalCommand(
    robot_pose, final_goal, /*xy_tolerance=*/0.20, /*yaw_tolerance=*/0.17, cmd);
  EXPECT_TRUE(arrived_xy);
  EXPECT_DOUBLE_EQ(cmd.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(cmd.angular.z, 0.0);
}

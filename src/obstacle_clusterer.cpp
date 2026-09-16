#include "nav2_orca_controller/obstacle_clusterer.hpp"
#include <cmath>
#include <limits>
#include <algorithm>
#include "nav2_util/node_utils.hpp"

namespace nav2_orca_controller
{

ObstacleClusterer::ObstacleClusterer(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
: parent_(parent), tf_buffer_(tf_buffer), costmap_ros_(costmap_ros)
{
  auto node = parent_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node in ObstacleClusterer");
  }

  // Clustering Parameters
  nav2_util::declare_parameter_if_not_declared(
    node, "cluster_dist_threshold", rclcpp::ParameterValue(0.3));
  node->get_parameter("cluster_dist_threshold", cluster_dist_threshold_);

  nav2_util::declare_parameter_if_not_declared(
    node, "min_cluster_size", rclcpp::ParameterValue(3));
  node->get_parameter("min_cluster_size", min_cluster_size_);

  nav2_util::declare_parameter_if_not_declared(
    node, "max_obstacle_radius", rclcpp::ParameterValue(2.0));
  node->get_parameter("max_obstacle_radius", max_obstacle_radius_);

  std::string scan_topic;
  nav2_util::declare_parameter_if_not_declared(
    node, "scan_topic", rclcpp::ParameterValue("scan"));
  node->get_parameter("scan_topic", scan_topic);

  // Subscribe to LaserScan
  scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&ObstacleClusterer::processScan, this, std::placeholders::_1));
}

ObstacleClusterer::~ObstacleClusterer()
{
}

void ObstacleClusterer::processScan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  auto start_time = std::chrono::high_resolution_clock::now();
  std::vector<OrcaObstacle> new_obstacles;
  
  if (!tf_buffer_ || !costmap_ros_) return;

  std::string global_frame = costmap_ros_->getGlobalFrameID();

  // Try to get the transform from scan frame to global frame
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      global_frame, scan->header.frame_id, 
      tf2::TimePointZero, tf2::durationFromSec(0.5));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("ObstacleClusterer"), *parent_.lock()->get_clock(), 2000,
      "Could not transform %s to %s: %s", 
      scan->header.frame_id.c_str(), global_frame.c_str(), ex.what());
    return;
  }

  std::vector<geometry_msgs::msg::Point> global_points;
  global_points.reserve(scan->ranges.size());

  // Filter and transform valid points
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double range = scan->ranges[i];
    if (std::isnan(range) || std::isinf(range) || 
        range < scan->range_min || range > scan->range_max) {
      continue;
    }

    double angle = scan->angle_min + i * scan->angle_increment;
    geometry_msgs::msg::Point pt_scan;
    pt_scan.x = range * std::cos(angle);
    pt_scan.y = range * std::sin(angle);
    pt_scan.z = 0.0;

    geometry_msgs::msg::Point pt_global;
    tf2::doTransform(pt_scan, pt_global, transform);
    global_points.push_back(pt_global);
  }

  // Simple Euclidean Clustering
  if (global_points.empty()) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_scan_obstacles_.clear();
    return;
  }

  std::vector<std::vector<geometry_msgs::msg::Point>> clusters;
  std::vector<geometry_msgs::msg::Point> current_cluster;
  current_cluster.push_back(global_points[0]);

  for (size_t i = 1; i < global_points.size(); ++i) {
    double dx = global_points[i].x - global_points[i-1].x;
    double dy = global_points[i].y - global_points[i-1].y;
    double dist = std::hypot(dx, dy);

    if (dist < cluster_dist_threshold_) {
      current_cluster.push_back(global_points[i]);
    } else {
      if (current_cluster.size() >= static_cast<size_t>(min_cluster_size_)) {
        clusters.push_back(current_cluster);
      }
      current_cluster.clear();
      current_cluster.push_back(global_points[i]);
    }
  }
  if (current_cluster.size() >= static_cast<size_t>(min_cluster_size_)) {
    clusters.push_back(current_cluster);
  }

  // Convert clusters to OrcaObstacles (Circles)
  for (const auto & cluster : clusters) {
    double sum_x = 0, sum_y = 0;
    for (const auto & pt : cluster) {
      sum_x += pt.x;
      sum_y += pt.y;
    }
    double center_x = sum_x / cluster.size();
    double center_y = sum_y / cluster.size();

    double max_dist_sq = 0;
    for (const auto & pt : cluster) {
      double dx = pt.x - center_x;
      double dy = pt.y - center_y;
      double dist_sq = dx*dx + dy*dy;
      if (dist_sq > max_dist_sq) {
        max_dist_sq = dist_sq;
      }
    }
    
    double radius = std::sqrt(max_dist_sq) + 0.05; // 5cm padding
    if (radius > max_obstacle_radius_) {
      radius = max_obstacle_radius_; // Cap radius
    }

    OrcaObstacle obs;
    obs.x = center_x;
    obs.y = center_y;
    obs.radius = radius;
    obs.is_dynamic = false; // KF + MHT tracking can be added here in the future
    
    new_obstacles.push_back(obs);
  }

  std::lock_guard<std::mutex> lock(mutex_);
  current_scan_obstacles_ = new_obstacles;

  auto end_time = std::chrono::high_resolution_clock::now();
  last_laser_process_time_ = std::chrono::duration<double, std::milli>(end_time - start_time).count();
}

void ObstacleClusterer::processCostmap()
{
  if (!costmap_ros_) return;

  auto costmap = costmap_ros_->getCostmap();
  if (!costmap) return;

  std::vector<OrcaObstacle> new_obstacles;
  
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock_cm(*(costmap->getMutex()));

  unsigned int size_x = costmap->getSizeInCellsX();
  unsigned int size_y = costmap->getSizeInCellsY();
  double resolution = costmap->getResolution();
  
  // To avoid adding thousands of cells, we could only extract boundary points, 
  // or group them. For simplicity and correctness with RVO2, we can add LETHAL cells
  // as small static obstacles.
  for (unsigned int y = 0; y < size_y; ++y) {
    for (unsigned int x = 0; x < size_x; ++x) {
      if (costmap->getCost(x, y) == nav2_costmap_2d::LETHAL_OBSTACLE) {
        double wx, wy;
        costmap->mapToWorld(x, y, wx, wy);
        
        OrcaObstacle obs;
        obs.x = wx;
        obs.y = wy;
        obs.radius = resolution / 2.0 + 0.01; // minimal padding
        obs.is_dynamic = false;
        
        new_obstacles.push_back(obs);
      }
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  current_costmap_obstacles_ = new_obstacles;
}

std::vector<OrcaObstacle> ObstacleClusterer::getObstacles()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<OrcaObstacle> all_obstacles = current_scan_obstacles_;
  all_obstacles.insert(all_obstacles.end(), 
                       current_costmap_obstacles_.begin(), 
                       current_costmap_obstacles_.end());
  return all_obstacles;
}

}  // namespace nav2_orca_controller

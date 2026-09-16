#ifndef NAV2_ORCA_CONTROLLER__OBSTACLE_CLUSTERER_HPP_
#define NAV2_ORCA_CONTROLLER__OBSTACLE_CLUSTERER_HPP_

#include <vector>
#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

namespace nav2_orca_controller
{

/**
 * @struct OrcaObstacle
 * @brief Representation of an obstacle for the ORCA simulator.
 */
struct OrcaObstacle {
  double x;
  double y;
  double radius;
  double vx = 0.0;
  double vy = 0.0;
  bool is_dynamic = false;
};

/**
 * @class ObstacleClusterer
 * @brief Processes LaserScans and Costmaps to extract distinct obstacle clusters for RVO2.
 * 
 * Designed as a dedicated class so that future expansions (e.g., Kalman Filter + MHT tracking)
 * can be easily swapped in for dynamic obstacle tracking.
 */
class ObstacleClusterer
{
public:
  ObstacleClusterer(rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
                    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);
  ~ObstacleClusterer();

  /**
   * @brief Process an incoming laser scan to extract distinct clusters
   */
  void processScan(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  /**
   * @brief Extract obstacles directly from the costmap (static obstacles)
   */
  void processCostmap();

  /**
   * @brief Get the latest combined list of obstacles
   */
  std::vector<OrcaObstacle> getObstacles();

  double getLastLaserProcessTime() const { return last_laser_process_time_; }

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  std::vector<OrcaObstacle> current_scan_obstacles_;
  std::vector<OrcaObstacle> current_costmap_obstacles_;
  std::mutex mutex_;

  // Node parameters for clustering
  double cluster_dist_threshold_;
  int min_cluster_size_;
  double max_obstacle_radius_;

  double last_laser_process_time_ = 0.0;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

}  // namespace nav2_orca_controller

#endif  // NAV2_ORCA_CONTROLLER__OBSTACLE_CLUSTERER_HPP_

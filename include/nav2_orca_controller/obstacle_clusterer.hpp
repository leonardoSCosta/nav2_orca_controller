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
#include "geometry_msgs/msg/point.hpp"

namespace nav2_orca_controller
{

/**
 * @struct OrcaObstacle
 * @brief Circular representation of an obstacle/agent for the ORCA simulator.
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
 * @struct OrcaPolygon
 * @brief Polygonal representation of a static obstacle (vertices in CCW order) for RVO2::addObstacle.
 */
struct OrcaPolygon {
  std::vector<geometry_msgs::msg::Point> vertices;
};

/**
 * @class ObstacleClusterer
 * @brief Processes LaserScans and Costmaps to extract distinct circular and polygonal obstacle representations for RVO2.
 */
class ObstacleClusterer
{
public:
  ObstacleClusterer(rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
                    const std::string & name,
                    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);
  ~ObstacleClusterer();

  /**
   * @brief Process an incoming laser scan to extract distinct clusters
   */
  void processScan(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  /**
   * @brief Extract static obstacle polygons (and small circular obstacles) directly from the costmap
   */
  void processCostmap();

  /**
   * @brief Get the latest combined list of circular obstacles
   */
  std::vector<OrcaObstacle> getObstacles();

  /**
   * @brief Get the latest list of static polygonal obstacles extracted from the costmap
   */
  std::vector<OrcaPolygon> getPolygons();

  double getLastLaserProcessTime() const { return last_laser_process_time_; }

  /**
   * @brief Recursively subdivide a point cluster so that every resulting circular obstacle
   * has radius <= max_radius (plus padding) and covers all points.
   */
  static void subdivideCluster(
    const std::vector<geometry_msgs::msg::Point> & cluster,
    double max_radius,
    double padding,
    std::vector<OrcaObstacle> & out_obstacles);

  /**
   * @brief Extract CCW polygonal contours and small circular clusters from a Costmap2D.
   */
  static void extractFromCostmap(
    nav2_costmap_2d::Costmap2D * costmap,
    int min_polygon_cells,
    double max_obstacle_radius,
    double simplification_tolerance,
    std::vector<OrcaPolygon> & out_polygons,
    std::vector<OrcaObstacle> & out_small_obstacles);

  /**
   * @brief Ramer-Douglas-Peucker simplification for a closed polygon ring.
   */
  static std::vector<geometry_msgs::msg::Point> simplifyClosedPolygon(
    const std::vector<geometry_msgs::msg::Point> & ring,
    double tolerance);

  /**
   * @brief Signed area of a 2D polygon (> 0 means counter-clockwise / CCW).
   */
  static double computeSignedArea(const std::vector<geometry_msgs::msg::Point> & pts);

private:
  bool isCoveredByCostmapLethal(
    const geometry_msgs::msg::Point & pt_global,
    nav2_costmap_2d::Costmap2D * costmap) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  std::vector<OrcaObstacle> current_scan_obstacles_;
  std::vector<OrcaObstacle> current_costmap_obstacles_;
  std::vector<OrcaPolygon> current_costmap_polygons_;
  std::mutex mutex_;

  // Node parameters for clustering
  double cluster_dist_threshold_;
  int min_cluster_size_;
  double max_obstacle_radius_;
  bool filter_scan_by_costmap_;
  int costmap_lethal_radius_padding_;
  double polygon_simplification_tolerance_;
  int min_polygon_cells_;

  double last_laser_process_time_ = 0.0;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

}  // namespace nav2_orca_controller

#endif  // NAV2_ORCA_CONTROLLER__OBSTACLE_CLUSTERER_HPP_

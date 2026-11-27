#include "nav2_orca_controller/obstacle_clusterer.hpp"
#include <cmath>
#include <limits>
#include <algorithm>
#include <queue>
#include <unordered_map>
#include <utility>
#include "nav2_util/node_utils.hpp"

namespace nav2_orca_controller
{

ObstacleClusterer::ObstacleClusterer(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
  const std::string & name,
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
    node, name + ".cluster_dist_threshold", rclcpp::ParameterValue(0.3));
  node->get_parameter(name + ".cluster_dist_threshold", cluster_dist_threshold_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".min_cluster_size", rclcpp::ParameterValue(3));
  node->get_parameter(name + ".min_cluster_size", min_cluster_size_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_obstacle_radius", rclcpp::ParameterValue(0.25));
  node->get_parameter(name + ".max_obstacle_radius", max_obstacle_radius_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".filter_scan_by_costmap", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".filter_scan_by_costmap", filter_scan_by_costmap_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".costmap_lethal_radius_padding", rclcpp::ParameterValue(1));
  node->get_parameter(name + ".costmap_lethal_radius_padding", costmap_lethal_radius_padding_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".polygon_simplification_tolerance", rclcpp::ParameterValue(0.05));
  node->get_parameter(name + ".polygon_simplification_tolerance", polygon_simplification_tolerance_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".min_polygon_cells", rclcpp::ParameterValue(3));
  node->get_parameter(name + ".min_polygon_cells", min_polygon_cells_);

  std::string scan_topic;
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".scan_topic", rclcpp::ParameterValue("scan"));
  node->get_parameter(name + ".scan_topic", scan_topic);

  // Subscribe to LaserScan
  scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&ObstacleClusterer::processScan, this, std::placeholders::_1));
}

ObstacleClusterer::~ObstacleClusterer()
{
}

bool ObstacleClusterer::isCoveredByCostmapLethal(
  const geometry_msgs::msg::Point & pt_global,
  nav2_costmap_2d::Costmap2D * costmap) const
{
  if (!costmap) {
    return false;
  }

  unsigned int mx = 0, my = 0;
  if (!costmap->worldToMap(pt_global.x, pt_global.y, mx, my)) {
    return false;
  }

  const int size_x = static_cast<int>(costmap->getSizeInCellsX());
  const int size_y = static_cast<int>(costmap->getSizeInCellsY());
  const int pad = std::max(0, costmap_lethal_radius_padding_);

  for (int dy = -pad; dy <= pad; ++dy) {
    int ny = static_cast<int>(my) + dy;
    if (ny < 0 || ny >= size_y) {
      continue;
    }
    for (int dx = -pad; dx <= pad; ++dx) {
      int nx = static_cast<int>(mx) + dx;
      if (nx < 0 || nx >= size_x) {
        continue;
      }
      if (costmap->getCost(static_cast<unsigned int>(nx), static_cast<unsigned int>(ny)) ==
        nav2_costmap_2d::LETHAL_OBSTACLE)
      {
        return true;
      }
    }
  }
  return false;
}

void ObstacleClusterer::subdivideCluster(
  const std::vector<geometry_msgs::msg::Point> & cluster,
  double max_radius,
  double padding,
  std::vector<OrcaObstacle> & out_obstacles)
{
  if (cluster.empty()) {
    return;
  }

  double sum_x = 0.0, sum_y = 0.0;
  for (const auto & pt : cluster) {
    sum_x += pt.x;
    sum_y += pt.y;
  }
  const double center_x = sum_x / static_cast<double>(cluster.size());
  const double center_y = sum_y / static_cast<double>(cluster.size());

  double max_dist_sq = 0.0;
  size_t far_idx_a = 0;
  for (size_t i = 0; i < cluster.size(); ++i) {
    const double dx = cluster[i].x - center_x;
    const double dy = cluster[i].y - center_y;
    const double dist_sq = dx * dx + dy * dy;
    if (dist_sq > max_dist_sq) {
      max_dist_sq = dist_sq;
      far_idx_a = i;
    }
  }

  const double radius = std::sqrt(max_dist_sq) + padding;

  // Base case: cluster fits within max_radius or cannot be split further
  if (radius <= max_radius || cluster.size() <= 2) {
    OrcaObstacle obs;
    obs.x = center_x;
    obs.y = center_y;
    obs.radius = std::min(radius, max_radius);
    obs.is_dynamic = false;
    out_obstacles.push_back(obs);
    return;
  }

  // Find point B farthest from point A to determine principal split axis
  const auto & pt_a = cluster[far_idx_a];
  double max_ab_sq = 0.0;
  size_t far_idx_b = far_idx_a;
  for (size_t i = 0; i < cluster.size(); ++i) {
    const double dx = cluster[i].x - pt_a.x;
    const double dy = cluster[i].y - pt_a.y;
    const double d_sq = dx * dx + dy * dy;
    if (d_sq > max_ab_sq) {
      max_ab_sq = d_sq;
      far_idx_b = i;
    }
  }

  if (max_ab_sq < 1e-8) {
    OrcaObstacle obs;
    obs.x = center_x;
    obs.y = center_y;
    obs.radius = std::min(radius, max_radius);
    obs.is_dynamic = false;
    out_obstacles.push_back(obs);
    return;
  }

  const auto & pt_b = cluster[far_idx_b];
  const double axis_x = pt_b.x - pt_a.x;
  const double axis_y = pt_b.y - pt_a.y;
  const double mid_x = 0.5 * (pt_a.x + pt_b.x);
  const double mid_y = 0.5 * (pt_a.y + pt_b.y);

  std::vector<geometry_msgs::msg::Point> left_cluster;
  std::vector<geometry_msgs::msg::Point> right_cluster;
  left_cluster.reserve(cluster.size());
  right_cluster.reserve(cluster.size());

  for (const auto & pt : cluster) {
    const double proj = (pt.x - mid_x) * axis_x + (pt.y - mid_y) * axis_y;
    if (proj <= 0.0) {
      left_cluster.push_back(pt);
    } else {
      right_cluster.push_back(pt);
    }
  }

  // Fallback split by index if all points ended up on one side
  if (left_cluster.empty() || right_cluster.empty()) {
    const size_t half = cluster.size() / 2;
    left_cluster.assign(cluster.begin(), cluster.begin() + static_cast<std::ptrdiff_t>(half));
    right_cluster.assign(cluster.begin() + static_cast<std::ptrdiff_t>(half), cluster.end());
  }

  subdivideCluster(left_cluster, max_radius, padding, out_obstacles);
  subdivideCluster(right_cluster, max_radius, padding, out_obstacles);
}

void ObstacleClusterer::processScan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  auto start_time = std::chrono::high_resolution_clock::now();
  std::vector<OrcaObstacle> new_obstacles;

  if (!tf_buffer_ || !costmap_ros_) {
    return;
  }

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

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_ptr<std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t>> cm_lock;
  if (filter_scan_by_costmap_ && costmap) {
    cm_lock = std::make_unique<std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t>>(
      *(costmap->getMutex()));
  }

  std::vector<geometry_msgs::msg::Point> global_points;
  global_points.reserve(scan->ranges.size());

  // Filter and transform valid points
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double range = scan->ranges[i];
    if (std::isnan(range) || std::isinf(range) ||
      range < scan->range_min || range > scan->range_max)
    {
      continue;
    }

    double angle = scan->angle_min + i * scan->angle_increment;
    geometry_msgs::msg::Point pt_scan;
    pt_scan.x = range * std::cos(angle);
    pt_scan.y = range * std::sin(angle);
    pt_scan.z = 0.0;

    geometry_msgs::msg::Point pt_global;
    tf2::doTransform(pt_scan, pt_global, transform);

    if (filter_scan_by_costmap_ && costmap && isCoveredByCostmapLethal(pt_global, costmap)) {
      continue;
    }

    global_points.push_back(pt_global);
  }

  if (cm_lock) {
    cm_lock->unlock();
  }

  // Simple Euclidean Clustering
  if (global_points.empty()) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_scan_obstacles_.clear();
    auto end_time = std::chrono::high_resolution_clock::now();
    last_laser_process_time_ =
      std::chrono::duration<double, std::milli>(end_time - start_time).count();
    return;
  }

  std::vector<std::vector<geometry_msgs::msg::Point>> clusters;
  std::vector<geometry_msgs::msg::Point> current_cluster;
  current_cluster.push_back(global_points[0]);

  for (size_t i = 1; i < global_points.size(); ++i) {
    double dx = global_points[i].x - global_points[i - 1].x;
    double dy = global_points[i].y - global_points[i - 1].y;
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

  // Convert clusters to OrcaObstacles (subdividing large clusters into multiple circles)
  for (const auto & cluster : clusters) {
    subdivideCluster(cluster, max_obstacle_radius_, 0.05, new_obstacles);
  }

  std::lock_guard<std::mutex> lock(mutex_);
  current_scan_obstacles_ = std::move(new_obstacles);

  auto end_time = std::chrono::high_resolution_clock::now();
  last_laser_process_time_ =
    std::chrono::duration<double, std::milli>(end_time - start_time).count();
}

namespace
{

double pointToSegmentDist(
  const geometry_msgs::msg::Point & p,
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b)
{
  const double vx = b.x - a.x;
  const double vy = b.y - a.y;
  const double len_sq = vx * vx + vy * vy;
  if (len_sq < 1e-12) {
    return std::hypot(p.x - a.x, p.y - a.y);
  }
  double t = ((p.x - a.x) * vx + (p.y - a.y) * vy) / len_sq;
  t = std::clamp(t, 0.0, 1.0);
  const double proj_x = a.x + t * vx;
  const double proj_y = a.y + t * vy;
  return std::hypot(p.x - proj_x, p.y - proj_y);
}

void rdpRecursive(
  const std::vector<geometry_msgs::msg::Point> & pts,
  size_t first,
  size_t last,
  double eps,
  std::vector<bool> & keep)
{
  if (last <= first + 1) {
    return;
  }
  double max_dist = -1.0;
  size_t split_idx = first;
  for (size_t i = first + 1; i < last; ++i) {
    double d = pointToSegmentDist(pts[i], pts[first], pts[last]);
    if (d > max_dist) {
      max_dist = d;
      split_idx = i;
    }
  }
  if (max_dist > eps) {
    keep[split_idx] = true;
    rdpRecursive(pts, first, split_idx, eps, keep);
    rdpRecursive(pts, split_idx, last, eps, keep);
  }
}

}  // namespace

double ObstacleClusterer::computeSignedArea(const std::vector<geometry_msgs::msg::Point> & pts)
{
  if (pts.size() < 3) {
    return 0.0;
  }
  double area = 0.0;
  for (size_t i = 0; i < pts.size(); ++i) {
    const auto & p1 = pts[i];
    const auto & p2 = pts[(i + 1) % pts.size()];
    area += (p1.x * p2.y - p2.x * p1.y);
  }
  return 0.5 * area;
}

std::vector<geometry_msgs::msg::Point> ObstacleClusterer::simplifyClosedPolygon(
  const std::vector<geometry_msgs::msg::Point> & ring,
  double tolerance)
{
  if (ring.size() <= 4 || tolerance <= 0.0) {
    return ring;
  }

  // First remove strictly collinear intermediate vertices along grid edges
  std::vector<geometry_msgs::msg::Point> corners;
  corners.reserve(ring.size());
  const size_t n = ring.size();
  for (size_t i = 0; i < n; ++i) {
    const auto & prev = ring[(i + n - 1) % n];
    const auto & curr = ring[i];
    const auto & next = ring[(i + 1) % n];
    const double dx1 = curr.x - prev.x;
    const double dy1 = curr.y - prev.y;
    const double dx2 = next.x - curr.x;
    const double dy2 = next.y - curr.y;
    const double cross = std::abs(dx1 * dy2 - dy1 * dx2);
    if (cross > 1e-9) {
      corners.push_back(curr);
    }
  }

  if (corners.size() <= 4) {
    return corners.size() >= 3 ? corners : ring;
  }

  // Find pair of vertices farthest apart to split closed ring into two open chains for RDP
  size_t idx_a = 0;
  double max_d = -1.0;
  for (size_t i = 1; i < corners.size(); ++i) {
    double d = std::hypot(corners[i].x - corners[0].x, corners[i].y - corners[0].y);
    if (d > max_d) {
      max_d = d;
      idx_a = i;
    }
  }
  size_t idx_b = 0;
  max_d = -1.0;
  for (size_t i = 0; i < corners.size(); ++i) {
    double d = std::hypot(corners[i].x - corners[idx_a].x, corners[i].y - corners[idx_a].y);
    if (d > max_d) {
      max_d = d;
      idx_b = i;
    }
  }

  if (idx_a == idx_b) {
    return corners;
  }

  // Reorder starting at idx_a
  std::vector<geometry_msgs::msg::Point> ordered;
  ordered.reserve(corners.size() + 1);
  size_t split_pos = 0;
  for (size_t k = 0; k < corners.size(); ++k) {
    size_t idx = (idx_a + k) % corners.size();
    if (idx == idx_b) {
      split_pos = k;
    }
    ordered.push_back(corners[idx]);
  }
  ordered.push_back(corners[idx_a]);  // close ring at the end

  std::vector<bool> keep(ordered.size(), false);
  keep[0] = true;
  keep[split_pos] = true;
  keep[ordered.size() - 1] = true;

  rdpRecursive(ordered, 0, split_pos, tolerance, keep);
  rdpRecursive(ordered, split_pos, ordered.size() - 1, tolerance, keep);

  std::vector<geometry_msgs::msg::Point> simplified;
  for (size_t i = 0; i + 1 < ordered.size(); ++i) {
    if (keep[i]) {
      simplified.push_back(ordered[i]);
    }
  }

  return simplified.size() >= 3 ? simplified : corners;
}

void ObstacleClusterer::extractFromCostmap(
  nav2_costmap_2d::Costmap2D * costmap,
  int min_polygon_cells,
  double max_obstacle_radius,
  double simplification_tolerance,
  std::vector<OrcaPolygon> & out_polygons,
  std::vector<OrcaObstacle> & out_small_obstacles)
{
  out_polygons.clear();
  out_small_obstacles.clear();
  if (!costmap) {
    return;
  }

  const unsigned int size_x = costmap->getSizeInCellsX();
  const unsigned int size_y = costmap->getSizeInCellsY();
  const double resolution = costmap->getResolution();
  const double origin_x = costmap->getOriginX();
  const double origin_y = costmap->getOriginY();

  std::vector<uint8_t> visited(size_x * size_y, 0);
  auto cellIdx = [size_x](unsigned int x, unsigned int y) -> size_t {
      return static_cast<size_t>(y) * size_x + x;
    };

  auto isLethal = [costmap, size_x, size_y](int x, int y) -> bool {
      if (x < 0 || y < 0 ||
        x >= static_cast<int>(size_x) || y >= static_cast<int>(size_y))
      {
        return false;
      }
      return costmap->getCost(static_cast<unsigned int>(x), static_cast<unsigned int>(y)) ==
             nav2_costmap_2d::LETHAL_OBSTACLE;
    };

  const int dx4[4] = {1, -1, 0, 0};
  const int dy4[4] = {0, 0, 1, -1};

  // Encode grid corner vertex (vx in [0..size_x], vy in [0..size_y])
  const uint64_t stride_v = static_cast<uint64_t>(size_x) + 1ULL;
  auto encodeVertex = [stride_v](int vx, int vy) -> uint64_t {
      return static_cast<uint64_t>(vy) * stride_v + static_cast<uint64_t>(vx);
    };
  auto decodeVertex = [stride_v, origin_x, origin_y, resolution](uint64_t code) {
      int vy = static_cast<int>(code / stride_v);
      int vx = static_cast<int>(code % stride_v);
      geometry_msgs::msg::Point pt;
      pt.x = origin_x + vx * resolution;
      pt.y = origin_y + vy * resolution;
      pt.z = 0.0;
      return pt;
    };

  for (unsigned int y = 0; y < size_y; ++y) {
    for (unsigned int x = 0; x < size_x; ++x) {
      if (visited[cellIdx(x, y)] ||
        costmap->getCost(x, y) != nav2_costmap_2d::LETHAL_OBSTACLE)
      {
        continue;
      }

      // BFS to collect 4-connected component of LETHAL cells
      std::vector<std::pair<int, int>> component;
      std::queue<std::pair<int, int>> q;
      q.push({static_cast<int>(x), static_cast<int>(y)});
      visited[cellIdx(x, y)] = 1;

      while (!q.empty()) {
        auto [cx, cy] = q.front();
        q.pop();
        component.push_back({cx, cy});

        for (int k = 0; k < 4; ++k) {
          int nx = cx + dx4[k];
          int ny = cy + dy4[k];
          if (nx >= 0 && ny >= 0 &&
            nx < static_cast<int>(size_x) && ny < static_cast<int>(size_y))
          {
            size_t n_idx = cellIdx(static_cast<unsigned int>(nx), static_cast<unsigned int>(ny));
            if (!visited[n_idx] &&
              costmap->getCost(
                static_cast<unsigned int>(nx),
                static_cast<unsigned int>(ny)) == nav2_costmap_2d::LETHAL_OBSTACLE)
            {
              visited[n_idx] = 1;
              q.push({nx, ny});
            }
          }
        }
      }

      // Small isolated blob -> represent as circular obstacle(s)
      if (static_cast<int>(component.size()) < min_polygon_cells) {
        std::vector<geometry_msgs::msg::Point> pts;
        pts.reserve(component.size());
        for (const auto & [cx, cy] : component) {
          double wx, wy;
          costmap->mapToWorld(static_cast<unsigned int>(cx), static_cast<unsigned int>(cy), wx, wy);
          geometry_msgs::msg::Point pt;
          pt.x = wx;
          pt.y = wy;
          pt.z = 0.0;
          pts.push_back(pt);
        }
        subdivideCluster(pts, max_obstacle_radius, resolution * 0.75, out_small_obstacles);
        continue;
      }

      // Build directed boundary edges around the component in CCW order
      // Each cell [cx, cx+1] x [cy, cy+1] contributes up to 4 directed edges
      // keeping the occupied cell on the left (CCW boundary):
      // Bottom (if (cx, cy-1) not in component): (cx, cy) -> (cx+1, cy)
      // Right  (if (cx+1, cy) not in component): (cx+1, cy) -> (cx+1, cy+1)
      // Top    (if (cx, cy+1) not in component): (cx+1, cy+1) -> (cx, cy+1)
      // Left   (if (cx-1, cy) not in component): (cx, cy+1) -> (cx, cy)
      //
      // Note: check against all LETHAL cells (since component is a maximal 4-connected
      // set of LETHAL cells, checking isLethal(neighbor) is equivalent to checking
      // membership in component).
      struct DirectedEdge {
        uint64_t to;
        int dir;  // 0: Right (+x), 1: Up (+y), 2: Left (-x), 3: Down (-y)
        bool used = false;
      };
      std::unordered_map<uint64_t, std::vector<DirectedEdge>> adj;
      adj.reserve(component.size() * 2);

      for (const auto & [cx, cy] : component) {
        if (!isLethal(cx, cy - 1)) {
          adj[encodeVertex(cx, cy)].push_back({encodeVertex(cx + 1, cy), 0, false});
        }
        if (!isLethal(cx + 1, cy)) {
          adj[encodeVertex(cx + 1, cy)].push_back({encodeVertex(cx + 1, cy + 1), 1, false});
        }
        if (!isLethal(cx, cy + 1)) {
          adj[encodeVertex(cx + 1, cy + 1)].push_back({encodeVertex(cx, cy + 1), 2, false});
        }
        if (!isLethal(cx - 1, cy)) {
          adj[encodeVertex(cx, cy + 1)].push_back({encodeVertex(cx, cy), 3, false});
        }
      }

      // Trace closed boundary loops; at pinch vertices (out-degree > 1),
      // turn right-most (CW turn: (prev_dir + 3) % 4) to keep outer boundary loops separated
      for (auto & [start_v, edges] : adj) {
        for (size_t e_idx = 0; e_idx < edges.size(); ++e_idx) {
          if (edges[e_idx].used) {
            continue;
          }

          std::vector<geometry_msgs::msg::Point> ring;
          uint64_t curr_v = start_v;
          int prev_dir = edges[e_idx].dir;
          edges[e_idx].used = true;
          ring.push_back(decodeVertex(curr_v));
          curr_v = edges[e_idx].to;

          size_t safety_counter = 0;
          const size_t max_steps = component.size() * 4 + 4;
          while (curr_v != start_v && safety_counter++ < max_steps) {
            ring.push_back(decodeVertex(curr_v));
            auto it = adj.find(curr_v);
            if (it == adj.end()) {
              break;
            }
            auto & out_edges = it->second;
            int chosen = -1;
            int best_priority = 10;
            for (size_t k = 0; k < out_edges.size(); ++k) {
              if (out_edges[k].used) {
                continue;
              }
              // Priority: right turn ((prev_dir + 3)%4) -> straight (prev_dir) -> left ((prev_dir+1)%4)
              int rel = (out_edges[k].dir - prev_dir + 4) % 4;
              int priority = (rel == 3) ? 0 : (rel == 0 ? 1 : (rel == 1 ? 2 : 3));
              if (priority < best_priority) {
                best_priority = priority;
                chosen = static_cast<int>(k);
              }
            }
            if (chosen < 0) {
              break;
            }
            out_edges[static_cast<size_t>(chosen)].used = true;
            prev_dir = out_edges[static_cast<size_t>(chosen)].dir;
            curr_v = out_edges[static_cast<size_t>(chosen)].to;
          }

          if (ring.size() < 3) {
            continue;
          }

          // Only keep outer boundary loops (signed area > 0 means CCW outer boundary;
          // internal holes have signed area < 0, and if a hole is needed RVO2 also
          // accepts CW vertices for negative obstacles!)
          double area = computeSignedArea(ring);
          if (std::abs(area) < 1e-6) {
            continue;
          }

          auto simplified = simplifyClosedPolygon(ring, simplification_tolerance);
          if (simplified.size() >= 3) {
            // Preserve original winding (CCW for outer obstacle, CW for internal hole)
            double simp_area = computeSignedArea(simplified);
            if ((area > 0.0 && simp_area < 0.0) || (area < 0.0 && simp_area > 0.0)) {
              std::reverse(simplified.begin(), simplified.end());
            }
            OrcaPolygon poly;
            poly.vertices = std::move(simplified);
            out_polygons.push_back(std::move(poly));
          }
        }
      }
    }
  }
}

void ObstacleClusterer::processCostmap()
{
  if (!costmap_ros_) {
    return;
  }

  auto costmap = costmap_ros_->getCostmap();
  if (!costmap) {
    return;
  }

  std::vector<OrcaPolygon> new_polygons;
  std::vector<OrcaObstacle> new_small_obstacles;

  {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock_cm(*(costmap->getMutex()));
    extractFromCostmap(
      costmap,
      min_polygon_cells_,
      max_obstacle_radius_,
      polygon_simplification_tolerance_,
      new_polygons,
      new_small_obstacles);
  }

  std::lock_guard<std::mutex> lock(mutex_);
  current_costmap_polygons_ = std::move(new_polygons);
  current_costmap_obstacles_ = std::move(new_small_obstacles);
}

std::vector<OrcaObstacle> ObstacleClusterer::getObstacles()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<OrcaObstacle> all_obstacles = current_scan_obstacles_;
  all_obstacles.insert(
    all_obstacles.end(),
    current_costmap_obstacles_.begin(),
    current_costmap_obstacles_.end());
  return all_obstacles;
}

std::vector<OrcaPolygon> ObstacleClusterer::getPolygons()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_costmap_polygons_;
}

}  // namespace nav2_orca_controller

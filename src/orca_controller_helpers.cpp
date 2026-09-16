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

  // 1. Obstacles as Cylinders (Circles in 2D)
  visualization_msgs::msg::Marker obs_marker;
  obs_marker.header.frame_id = costmap_ros_->getGlobalFrameID();
  obs_marker.header.stamp = stamp;
  obs_marker.ns = "obstacles";
  obs_marker.id = 0;
  obs_marker.type = visualization_msgs::msg::Marker::CYLINDER;
  obs_marker.action = visualization_msgs::msg::Marker::ADD;
  
  for (size_t i = 0; i < obstacles.size(); ++i) {
    const auto & obs = obstacles[i];
    obs_marker.id = i;
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

  // To delete old markers, we can add a delete all marker, or rely on namespace.
  // A cleaner way is to keep track of previous count and delete excess, but for simplicity:
  visualization_msgs::msg::Marker del_marker;
  del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  // Note: DELETEALL is not supported in some older RViz, but fine in ROS 2. 
  // Let's just push it as the first marker so it clears before adding new ones.
  // Actually, wait, DELETEALL deletes everything. Let's just rely on RViz marker lifetime or overwrite.
  // Let's set a lifetime.
  for (auto & m : marker_array.markers) {
    m.lifetime = rclcpp::Duration::from_seconds(0.5);
  }

  // 2. Velocity Command Vector (Arrow)
  visualization_msgs::msg::Marker vel_marker;
  vel_marker.header.frame_id = robot_pose.header.frame_id; // Usually base_link
  vel_marker.header.stamp = stamp;
  vel_marker.ns = "velocity_cmd";
  vel_marker.id = 0;
  vel_marker.type = visualization_msgs::msg::Marker::ARROW;
  vel_marker.action = visualization_msgs::msg::Marker::ADD;
  vel_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
  
  // Base of arrow at origin of robot frame
  geometry_msgs::msg::Point p_start;
  p_start.x = 0; p_start.y = 0; p_start.z = 0.1;
  
  geometry_msgs::msg::Point p_end;
  p_end.x = cmd_vel.twist.linear.x;
  p_end.y = cmd_vel.twist.linear.y;
  p_end.z = 0.1;

  vel_marker.points.push_back(p_start);
  vel_marker.points.push_back(p_end);

  vel_marker.scale.x = 0.05; // shaft diameter
  vel_marker.scale.y = 0.1;  // head diameter
  vel_marker.scale.z = 0.1;  // head length

  vel_marker.color.r = 0.0;
  vel_marker.color.g = 1.0;
  vel_marker.color.b = 0.0;
  vel_marker.color.a = 1.0;

  marker_array.markers.push_back(vel_marker);

  marker_pub_->publish(marker_array);
}

# nav2_orca_controller

This is a custom local trajectory planner (controller) for the ROS 2 Nav2 navigation stack based on the **ORCA (Optimal Reciprocal Collision Avoidance)** algorithm. It utilizes the standalone [RVO2 C++ Library](https://github.com/snape/RVO2) to provide fast, collision-free velocities in highly dynamic environments. 

By default, the controller respects differential-drive kinematics and safely integrates both static obstacles from the Nav2 `Costmap2D` and distinct obstacle clusters dynamically processed from 2D LiDAR scans.

## Features

- **Standard Nav2 Plugin:** Fully implements the `nav2_core::Controller` interface for seamless integration.
- **Dynamic LiDAR Clustering:** Groups raw `sensor_msgs/msg/LaserScan` points into obstacles to feed directly into the RVO2 simulator.
- **Costmap Awareness:** Automatically treats lethal costmap cells as static obstacles.
- **Kinematic Constraints:** Built-in differential drive mapping ensures compatibility with standard non-holonomic platforms (adjustable via `is_holonomic`).
- **Low Computational Cost:** The RVO2 engine solves velocities for multiple agents in less than a millisecond per cycle.
- **Built-in Diagnostics & Visualization:** Publishes timing diagnostics and RViz markers (arrows for velocity vectors, cylinders for obstacles) out of the box.

## Configuration

To use `nav2_orca_controller` in Nav2, add it as a plugin in your `nav2_params.yaml` file under the `controller_server` node (see the provided example in [config/orca_controller.yaml](config/orca_controller.yaml)):

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    
    FollowPath:
      plugin: "nav2_orca_controller::OrcaController"
      # Insert your parameters here (see table below)
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `time_step` | double | 0.1 | The time step of the simulation in seconds. |
| `neighbor_dist` | double | 5.0 | Max distance (meters) to other agents/obstacles the robot takes into account. |
| `max_neighbors` | int | 10 | Max number of other agents/obstacles taken into account. |
| `time_horizon` | double | 3.0 | Minimal amount of time for which computed velocities are safe w.r.t agents. |
| `time_horizon_obst` | double | 1.5 | Minimal amount of time for which computed velocities are safe w.r.t static obstacles. |
| `robot_radius` | double | 0.22 | Radius of the robot (meters). |
| `safety_buffer` | double | 0.10 | Extra padding added to obstacle/agent radiuses. |
| `max_speed` | double | 0.5 | Maximum linear speed of the robot (m/s). |
| `lookahead_dist` | double | 0.45 | Distance on the global plan to target (meters). |
| `is_holonomic` | bool | false | Set to true if the robot is omnidirectional. If false, diff-drive mapping is applied. |
| `kw` | double | 1.8 | Proportional gain for angular velocity correction. |
| `max_angular_vel` | double | 0.8 | Maximum rotational speed (rad/s). |
| `rotate_to_goal_heading` | bool | true | When within `goal_dist_tolerance`, stops linear motion and rotates in place to final goal orientation. |
| `goal_dist_tolerance` | double | 0.20 | Fallback XY distance tolerance (meters) to switch to final goal orientation alignment. |
| `goal_yaw_tolerance` | double | 0.15 | Fallback yaw tolerance (rad) for final goal alignment. |
| `goal_decel_distance` | double | 0.50 | Distance from final goal (meters) at which linear speed smoothly decelerates. |
| `min_approach_speed` | double | 0.06 | Minimum linear approach speed (m/s) during final goal deceleration. |
| `debug_mode` | bool | false | Enables publishing RViz debug markers on `orca_markers` (obstacles, static polygons, and curved trajectory arrow). |
| `allow_short_reversal` | bool | true | Allows short emergency backward maneuvers when desired velocity points sharply backward. |
| `min_backward_angle` | double | 2.44 | Minimum heading error (rad, ~140°) required to trigger a short backward maneuver. |
| `max_backward_distance` | double | 0.35 | Maximum continuous backward travel distance (meters) before forcing an in-place forward turn. |
| `max_backward_duration` | double | 1.5 | Maximum continuous backward travel duration (seconds) before forcing an in-place forward turn. |
| `max_backward_speed` | double | 0.15 | Maximum linear speed (m/s) when backing up. |
| `forward_turn_slowdown_angle` | double | 1.22 | Heading error (rad, ~70°) above which forward speed scales to zero to rotate in place. |
| `cluster_dist_threshold`| double | 0.3 | Max Euclidean distance between two LiDAR points to belong to the same cluster. |
| `min_cluster_size` | int | 3 | Minimum number of points required to form a valid obstacle cluster. |
| `max_obstacle_radius` | double | 0.25 | Maximum allowed radius for a circular obstacle cluster (larger clusters are recursively subdivided into multiple circles). |
| `filter_scan_by_costmap` | bool | true | Filters out LiDAR points already covered by `LETHAL_OBSTACLE` cells in the costmap. |
| `costmap_lethal_radius_padding` | int | 1 | Cell radius margin around lethal costmap cells when filtering redundant LiDAR points. |
| `polygon_simplification_tolerance` | double | 0.05 | Ramer-Douglas-Peucker simplification tolerance (meters) for static costmap polygons. |
| `min_polygon_cells` | int | 3 | Minimum connected lethal cells to form a polygon (smaller blobs are represented as circles). |
| `scan_topic` | string | "scan" | Topic name to subscribe to for raw `sensor_msgs/msg/LaserScan` data. |

## Topics

### Subscribed Topics
- `<scan_topic>` (`sensor_msgs/msg/LaserScan`): Raw LiDAR scans used to dynamically cluster obstacles.

### Published Topics
- `orca_markers` (`visualization_msgs/msg/MarkerArray`): Published when `debug_mode` is `true`, including:
  - Red/Orange cylinders representing static and dynamic obstacles considered by ORCA.
  - A Green curved arrow representing the predicted trajectory (length proportional to linear speed and curved laterally by angular velocity).
- `~/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`): Publishes computation times in milliseconds for LiDAR clustering, Costmap processing, and the RVO2 simulation step.

## Build Requirements
This package uses CMake's `FetchContent` to automatically download and link the [snape/RVO2](https://github.com/snape/RVO2) library at build time (tag `v2.0.2`). No manual git submodules or external dependencies are required besides standard ROS 2 Humble packages.

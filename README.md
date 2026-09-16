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

To use `nav2_orca_controller` in Nav2, add it as a plugin in your `nav2_params.yaml` file under the `controller_server` node:

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
| `lookahead_dist` | double | 0.8 | Distance on the global plan to target (meters). |
| `is_holonomic` | bool | false | Set to true if the robot is omnidirectional. If false, diff-drive mapping is applied. |
| `kw` | double | 1.5 | Proportional gain for angular velocity correction. |
| `cluster_dist_threshold`| double | 0.3 | Max Euclidean distance between two LiDAR points to belong to the same cluster. |
| `min_cluster_size` | int | 3 | Minimum number of points required to form a valid obstacle cluster. |
| `max_obstacle_radius` | double | 2.0 | Maximum allowed radius for an obstacle cluster. |
| `scan_topic` | string | "scan" | Topic name to subscribe to for raw `sensor_msgs/msg/LaserScan` data. |

## Topics

### Subscribed Topics
- `<scan_topic>` (`sensor_msgs/msg/LaserScan`): Raw LiDAR scans used to dynamically cluster obstacles.

### Published Topics
- `orca_markers` (`visualization_msgs/msg/MarkerArray`): Publishes RViz markers for debugging, including:
  - Red/Orange cylinders representing static and dynamic obstacles.
  - A Green arrow representing the chosen output velocity command.
- `~/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`): Publishes computation times in milliseconds for LiDAR clustering, Costmap processing, and the RVO2 simulation step.

## Build Requirements
This package uses CMake's `FetchContent` to automatically download and link the [snape/RVO2](https://github.com/snape/RVO2) library at build time (tag `v2.0.2`). No manual git submodules or external dependencies are required besides standard ROS 2 Humble packages.

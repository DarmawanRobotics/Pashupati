# Patrol stack

ROS 2 Humble teach-and-repeat patrol for legged and wheeled robots: record a route once, mark
inspection stop points, then patrol it in a loop with reactive obstacle avoidance and precise
stops for anomaly inspection. Robot-agnostic; the robot driver only has to expose `cmd_vel`,
`battery` and a URDF with the Livox and camera frames.

| Package | Role |
|---|---|
| `robot_interfaces` | Messages and services (SectorScan, AvoidanceCommand, WaypointPath, StopPointEvent, ...) |
| `robot_mapping` | FAST-LIO2 mapping and route recording (`path_recorder_node`) |
| `robot_localization` | FAST-LIO frame bridge and map->odom from AprilTags / RViz |
| `robot_perception` | Livox points to a 2D sector scan, AprilTag detection |
| `robot_navigation` | Route loading/smoothing, path following, avoidance, safety |
| `robot_rviz` | RViz control panel |

## Data flow

```
livox/lidar --> lidar_sector_node --> perception/sector_scan --> obstacle_avoidance_node
                                                                        | navigation/avoidance
route.csv --> path_loader_node --> navigation/waypoints --> path_follower_node --> cmd_vel
FAST-LIO (camera_init->body) + localization_node (map->odom) ----TF----^
```

## Workflow

```bash
# 1. Map and record a route (drive the robot by hand)
ros2 launch robot_mapping mapping.launch.py
ros2 service call /mapping/path_record std_srvs/srv/SetBool "{data: true}"
ros2 service call /mapping/mark_stop_point robot_interfaces/srv/MarkStopPoint "{dwell_sec: 10.0}"
ros2 service call /mapping/path_record std_srvs/srv/SetBool "{data: false}"

# 2. Patrol (Genisom L1W example)
ros2 launch robot_localization localization.launch.py tags_config_file:=<tags.json>
ros2 launch robot_perception perception.launch.py
ros2 launch robot_navigation navigation.launch.py \
  waypoints_file:=<route.csv> cmd_vel_topic:=/l1w/cmd_vel battery_topic:=/l1w/battery
ros2 service call /navigation/start_nav std_srvs/srv/SetBool "{data: true}"
```

Close the route (end within `loop_close_distance` of the start) to patrol in a loop.

## Patrol behaviour

- **Route processing**: the recording is deduplicated, resampled and gradient-smoothed between
  stop points; stop points never move. Raw and smoothed routes are on `navigation/path_raw` and
  `navigation/path`.
- **Speed profile**: speed is capped by curvature and planned down to `approach_speed` before
  every stop point and the route end.
- **Controllers** (`controller`): `pure_pursuit`, `pid`, `mppi`, `lqr`, `stanley`.
- **Avoidance** (`algorithm`): `braitenberg`, `vfh`, `potential_field`, `follow_gap`. The
  potential field also pushes holonomic robots sideways (`linear.y`).
- **Stop points**: inside `approach_radius` the robot converges onto x/y/yaw (strafing when
  `holonomic: true`) to within 5 cm / 3 deg, calls every `inspection_services` Trigger, waits for
  them and dwells. Progress is reported on `navigation/stop_point_event`.
- **Safety**: stops on stale avoidance or odometry, emergency stop with release hysteresis,
  `BLOCKED` after `blocked_timeout_sec`, slow-down then stop when off the route, low-battery end
  of patrol, `navigation/pause` to hold without losing progress.

## Interfaces

| Name | Type | Node |
|---|---|---|
| `navigation/start_nav` | `std_srvs/SetBool` | path_follower_node |
| `navigation/pause` | `std_srvs/SetBool` | path_follower_node |
| `navigation/load_path` | `robot_interfaces/LoadPath` | path_loader_node |
| `navigation/status` | `robot_interfaces/NavigationStatus` | path_follower_node |
| `navigation/mission_status` | `robot_interfaces/MissionStatus` (1 Hz) | path_follower_node |
| `navigation/stop_point_event` | `robot_interfaces/StopPointEvent` | path_follower_node |
| `localization/start` | `std_srvs/Trigger` | localization_node |
| `mapping/path_record` | `std_srvs/SetBool` | path_recorder_node |
| `mapping/mark_stop_point` | `robot_interfaces/MarkStopPoint` | path_recorder_node |

All parameters are documented in `robot_navigation/config/navigation_params.yaml`.

## Tests

```bash
colcon test --packages-select robot_navigation && colcon test-result --verbose
```

The simulation tests drive every controller around a noisy closed loop and an overlapping
out-and-back corridor, and check path smoothing, the speed profile, the final approach and the
avoidance algorithms.

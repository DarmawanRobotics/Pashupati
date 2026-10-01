# robot_navigation

Route following for patrols.

| Node | Role |
|---|---|
| `path_loader_node` | loads a route CSV, resamples and smooths it between stop points, publishes it latched |
| `path_follower_node` | follows the route (controllers `pure_pursuit`, `pid`, `mppi`, `lqr`, `stanley`), speed profile, final x/y/yaw approach, inspection at stop points, loop patrol, safety supervisor |
| `obstacle_avoidance_node` | reactive avoidance on `SectorScan` (`braitenberg`, `vfh`, `potential_field`, `follow_gap`) |

Every parameter applies at runtime (Tuning panel or `ros2 param set`) except `control_rate` and the
service names.

| Interface | Type | |
|---|---|---|
| `navigation/start_nav`, `navigation/pause` | `std_srvs/SetBool` | run / hold |
| `navigation/load_path` | `robot_interfaces/LoadPath` | load a route |
| `navigation/status`, `navigation/mission_status` | status messages | state, lap, next stop |
| `navigation/stop_point_event` | `robot_interfaces/StopPointEvent` | arrived / inspected / departed / skipped |
| `navigation/markers`, `navigation/route_markers`, `navigation/avoidance_markers` | `MarkerArray` | debug drawing |
| `cmd_vel` | `geometry_msgs/Twist` | output (route it through `robot_bridge`) |

Parameters: [config/navigation_params.yaml](config/navigation_params.yaml).
`colcon test --packages-select robot_navigation` runs the controller/route simulations.

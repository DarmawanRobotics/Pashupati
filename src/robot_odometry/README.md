# robot_odometry

LiDAR-inertial odometry from FAST-LIO2 and the Livox Mid-360 (`camera_init → body`, `/Odometry`).
This is odometry only: it starts at the robot's pose on every launch and drifts slowly. `map → odom`
is set by [robot_localization](../robot_localization).

`odometry.launch.py` starts:

- **FAST-LIO** with [config/fast_lio.yaml](config/fast_lio.yaml), which is tuned for indoor use
  (0.3 m voxels, 4 iEKF iterations, 50 m range, no ever-growing `/path`).
- **`wait_still`**, the launch prefix of FAST-LIO. FAST-LIO estimates gravity and the IMU biases
  from its first ~10 IMU samples and assumes the robot is not moving during them. `wait_still`
  holds the start until the IMU has been still and level for `still_sec`, and it runs again on
  every respawn. Turn it off with `wait_still:=false`.
- **`odometry_monitor_node`**, which publishes `odometry/status`. It checks for:
  - stale `/Odometry`
  - jumps
  - z drift on a flat floor, which points to a tilted start
  - LIO speed that disagrees with the robot's own `/l1w/odom` for more than `speed_diff_sec`,
    which points to a degenerate corridor or slip
  - FAST-LIO restarts

| Topic | Type | |
|---|---|---|
| `/Odometry` | `nav_msgs/Odometry` | FAST-LIO, `camera_init → body` |
| `/cloud_registered_body` | `sensor_msgs/PointCloud2` | deskewed scan, feeds lidar_sector_node |
| `odometry/status` | `std_msgs/String`, latched | `initializing`, `ok`, `degraded: …`, `lost: …` |
| `odometry/restarted` | `std_msgs/Empty` | odom frame reset; `localization_node` drops `map → odom` |

```bash
ros2 launch robot_odometry odometry.launch.py              # patrol
ros2 launch robot_odometry odometry.launch.py save_map:=true
ros2 topic echo /odometry/status
```

Before a run, stand the robot up and keep it still for 2 s. If the status shows
`degraded: z drift`, restart odometry with the robot standing still. Gate and monitor thresholds
are in [config/odometry_params.yaml](config/odometry_params.yaml).

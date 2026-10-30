# xtras: third-party code and the robot package

Pashupati runs on any quadruped, legged or wheel-legged, that has a ROS 2 driver. All the
robot-specific parts live in one **robot package** that you add here: driver, URDF and sensor
launch. Nothing else in the repository depends on a robot brand.

| Path                        | What                                                                                                                                                                                           |
| --------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `drivers/livox_ros_driver2` | Livox Mid-360 driver (submodule)                                                                                                                                                               |
| `slam/fast_lio2`            | FAST-LIO2, `odometry:=fast_lio` (submodule)                                                                                                                                                    |
| `slam/leg_kilo`             | Leg-KILO, `odometry:=leg_kilo` (optional: clone the `pashupati` branch of the DarmawanRobotics fork here, see [dharma_odometry](https://github.com/DarmawanRobotics/dharma_odometry#leg-kilo)) |
| `robots/<name>`             | your robot package, added per deployment                                                                                                                                                       |

## Adding a robot

```bash
git submodule add <robot repo url> src/xtras/robots/<name>   # pinned version; or clone it there
scripts/setup.sh install rosdep build
```

Then set these in `/etc/pashupati/pashupati.env`:

| Variable         | Meaning                                                                                                                                            |
| ---------------- | -------------------------------------------------------------------------------------------------------------------------------------------------- |
| `ROBOT_DRIVER`   | `<package>[/<file>.launch.py]` (default file `bringup.launch.py`). It is started with `namespace:=<ROBOT_NS>`.                                     |
| `ROBOT_NS`       | namespace of the driver topics and services (default `robot`)                                                                                      |
| `ROBOT_PARAMS`   | the robot profile (`robot.*`), e.g. `<package>/config/robot.yaml`; empty: Pashupati's `config/robot.yaml`                                          |
| `ROBOT_SETUP`    | optional: the package's own installer (`<script> check\|install`), for a vendor SDK, udev rules, ... Run it with `scripts/setup.sh install robot`. |
| `ROBOT_PRESTART` | optional: a command run before the stack starts, e.g. to stop a vendor autostart that would fight for the robot                                    |
| `ROBOT_IP`       | optional: address of the robot computer, checked by `scripts/setup.sh check`                                                                       |

The robot itself is described by its robot profile (see [below](#the-robot-profile)).

## What the robot package must provide

[`dharma_sim`](https://github.com/DarmawanRobotics/dharma_sim) implements exactly this interface
for the simulator. Use it as the reference.

### Launch

`launch/<file>.launch.py` takes a `namespace` argument and starts these:

- the driver
- `robot_state_publisher` with the URDF
- the sensor drivers

It must not start any Pashupati node.

### Topics and services, all under `/<ROBOT_NS>/`

| Name                                     | Type                              | Direction | Contract                                                                                                                                                                                                                                                                             |
| ---------------------------------------- | --------------------------------- | --------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `cmd_vel`                                | `geometry_msgs/Twist`             | subscribe | Body frame (`base_link`): `linear.x` forward m/s, `linear.y` left m/s, `angular.z` yaw rad/s. Ignore `linear.y` if the robot cannot strafe. **Stop if no command arrives for 0.5 s.** Only `control_mux_node` publishes here, after the e-stop, speed envelope and protective field. |
| `odom`                                   | `nav_msgs/Odometry`               | publish   | The robot's own velocity estimate, ≥ 10 Hz. Pashupati uses `twist`, in `base_link`. Cross-checks the LiDAR odometry.                                                                                                                                                                 |
| `battery`                                | `sensor_msgs/BatteryState`        | publish   | `percentage` 0–1 and `voltage`, ≥ 0.2 Hz. Used for low-battery return and telemetry.                                                                                                                                                                                                 |
| `diagnostics`                            | `diagnostic_msgs/DiagnosticArray` | publish   | Optional: driver state for the handheld and NETRA.                                                                                                                                                                                                                                   |
| `stand_up`, `sit_down`, `emergency_stop` | `std_srvs/Trigger`                | serve     | The handheld buttons. Add more with `robot_commands`.                                                                                                                                                                                                                                |

Navigation can also call `std_srvs/Trigger` services of the driver when a patrol starts and
stops: `path_follower_node` `hooks.start` / `hooks.stop` in the site file. Use them on robots
that must hand motion control from the vendor remote to the SDK.

A robot with a charging dock serves the dock services `docking_node` calls (`hooks.sit`,
`hooks.charge`, `hooks.stop_charge`, `hooks.stand`, all `std_srvs/Trigger`; dharma_sim's dock is
the reference); the handheld's and NETRA's `dock` / `undock` then work.

### TF (URDF)

```
base_link ─┬─ livox_frame ── livox_imu_frame     (LiDAR and its IMU)
           └─ camera_link ── camera_*_frame      (RealSense driver, `ros-humble-realsense2-camera`)
```

- `base_link` is the centre of the body: x forward, z up (REP-103).
- `livox_imu_frame` is where FAST-LIO's `body` sits. `localization_node` needs `base_link → livox_imu_frame` before it starts.
- **The driver must not publish `odom → base_link`.** Pashupati owns `map → odom → camera_init → body → base_link`, so turn off the driver's odometry TF.

### Sensors

- **LiDAR: Livox Mid-360** through `livox_ros_driver2`, with `xfer_format: 1`. The cloud goes to `/livox/lidar`, the IMU to `/livox/imu`. FAST-LIO and the odometry start gate use both.
  - Another LiDAR needs its own FAST-LIO preprocess settings (`fastlio_mapping_node` in the site
    file, on top of dharma_odometry's `fast_lio.yaml`).
  - It also needs `lidar_sector_node` `input.type: pointcloud2`.
- **Camera** (AprilTag localization, VLM inspection, people, handheld video). For the RealSense the
  driver comes from apt (`scripts/setup.sh install realsense`: `ros-humble-realsense2-camera` and
  the udev rules); the robot package only launches it. It publishes:
  - `/camera/camera/color/image_raw`
  - `/camera/camera/color/image_raw/compressed`
  - `camera_info`

  An IP camera instead: `CAMERA=rtsp` and `CAMERA_URL` start `rtsp_camera_node`
  (pashupati_bridge), which publishes the same topics.

## The robot profile

Every node reads the robot from one profile, `robot.*` (`ROBOT_PARAMS`; Pashupati's
`src/pashupati/config/robot.yaml` is the field-tested L1W). Ship one in the robot package:

```yaml
/**:
  ros__parameters:
    robot:
      name: my_dog
      kinematics: omni # diff for a robot that cannot strafe (then max_vy 0)
      base_height: 0.35 # m, base_link above the floor when standing
      footprint: { front: 0.40, rear: 0.40, half_width: 0.28 } # m, body and legs
      limits:
        max_vx: 1.0 # m/s, nothing may command more
        max_vy: 0.5
        max_wz: 1.5 # rad/s
        max_accel: 0.5
        max_decel: 0.6 # m/s^2, MEASURE: braking from full speed on the site's floor
        max_yaw_accel: 1.5
        reaction_time: 0.25 # s, command to the robot actually braking
```

The protective field, the planner, path shift, the LiDAR self filter and height band, the
handheld's speed levels and the person detector's floor all follow from it. This robot's
handheld actions go in the site file: `teleop_udp_node` `robot_commands` (its own Trigger
services).

## Checklist for a new robot

1. **Robot package alone:**
   ```bash
   ros2 launch <pkg> <file> namespace:=<ns>
   ros2 topic hz /<ns>/odom
   ros2 topic hz /livox/lidar
   ros2 topic hz /livox/imu
   ros2 run tf2_ros tf2_echo base_link livox_imu_frame
   ```
2. **Motion and the watchdog:** with the robot lifted or in an open area, this should drive it
   slowly forward. When the publisher stops, the robot must stop within 0.5 s.
   ```bash
   ros2 topic pub -r 20 /<ns>/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
   ```
3. **Whole stack:**
   ```bash
   ros2 launch pashupati robot.launch.py robot_driver:=<pkg> robot_ns:=<ns> robot_params:=<profile.yaml>
   ros2 topic echo /health/status
   ```
   `/health/status` should report `level: ok` once odometry is running.
4. **Braking distance:** measure it, then set `robot.limits.max_decel` and
   `robot.limits.reaction_time` before the first patrol (Tuning Guide, section 3.2).

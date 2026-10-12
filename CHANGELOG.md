# Changelog

All notable changes to the Pashupati robot stack. Format: [Keep a Changelog](https://keepachangelog.com),
versions follow [SemVer](https://semver.org).

## [Unreleased]

### Fixed
- The control mux forwarded NaN / infinite velocities (the protective field computed a NaN
  scale and let them through); they are now dropped as zero, and every command is capped at
  the robot's speed envelope (`max_vx`, `max_vy`, `max_wz`).
- `path_follower_node` crashed on its first FOLLOWING → BLOCKED / OFF_PATH / EMERGENCY_STOP change
  (rclpy refuses to change a log call site's severity); the respawned node came back with
  navigation inactive.
- `teleop_udp_node` and `fleet_uplink_node` crashed at start (`handle` / `context` shadowed rclpy
  Node members); all nodes stop cleanly on Ctrl-C / SIGTERM.
- Teleop: NaN / inf / non-numeric joystick values were clamped to full speed, now 0; malformed
  datagrams no longer raise; one driver at a time (`busy`); token checked in constant time;
  `load_route` restricted to `routes_dir`.
- Fleet uplink: a bad spool item no longer kills the uploader (quarantined to `.failed/`); route ids
  from the server cannot escape `routes_dir`.
- `localization_node` crash-looped when the tag file did not exist yet.
- Route CSV rows with NaN, infinity or text are skipped and logged with their line number
  instead of loading waypoints no controller can follow; a negative dwell is no stop.
- `apriltag_node` aborted at start and respawned forever: `detector.refine` / `detector.debug`
  were integers, `apriltag_ros` declares them as booleans.
- `rosdep install` failed on `robot_localization` (`ament_python` is not a rosdep key).
- Low-battery flag never cleared after charging; inspection answers could be reported for the
  wrong stop.
- The anomaly photo sent to NETRA is now the frame the VLM analysed (`anomaly_detector/evidence`).
- No traceback from the TF listener thread when nodes stop.
- `map → odom` is a dynamic transform on `/tf` (REP-105), republished at `map_odom_hz` and after
  every relocalization. As a static transform it only worked on Humble: from Jazzy on, the static
  broadcaster keeps the first value, so relocalizing had no effect.
- A Ctrl-C that lands while a node builds its wait set no longer ends it with an `RCLError`
  traceback (exit 1); the teleop gateway frees its UDP port as soon as it stops.
- A stop whose inspection services are all down now reports `inspected` with `not available`
  right away; before, no inspection event was sent for it at all.
- `systemctl stop` / restart interrupted every node twice (systemd and `ros2 launch` both sent
  SIGINT), so nodes died mid-shutdown with a `KeyboardInterrupt`. The service now signals only
  `ros2 launch` (`KillMode=mixed`); re-run `script/setup.sh install service` to update the unit.

### Added
- Teleop LAN discovery (`discover` → `beacon` with name, token required, who is driving,
  mode, battery), a `seq` number on every `state` for packet-loss estimates, and a camera rate
  the handheld can lower (`camera_fps`, capped by the parameter).
- `robot_odometry`: tuned FAST-LIO config, stillness gate before IMU initialisation (also on
  respawn), odometry monitor (`odometry/status`, `odometry/restarted`), optional Leg-KILO, route
  recorder (`route/record`, `route/mark_stop_point`, `route/saved`).
- Site configuration `/etc/pashupati/site.yaml` applied on top of every node's defaults.
- Latched software e-stop in the control mux (`bridge/estop`, handheld `estop` / `emergency_stop`,
  NETRA `estop`), `ESTOP` alert to NETRA; the mux also zeroes the robot when navigation goes silent.
- Protective field (speed and separation monitoring) in the control mux: every command, navigation
  or handheld, in any direction, is limited to a speed that stops before the nearest obstacle in
  the swept path; 360° `perception/safety_scan` with LiDAR-blind detection.
- Health monitor: missing nodes (the launch passes the list it starts), lost odometry, stale safety
  scan and e-stop reported as critical (`level: error`) next to sensor rates and the computer.
- CI: build, unit + in-process ROS tests, lint (flake8, pep257, shellcheck, YAML) on every PR.
- `odometry:=external`: no estimator is started, `/Odometry` comes from elsewhere (the simulator);
  the odometry monitor and route recorder still run.
- `robot_sim`: the production stack on a simulated L1W (2D world, ray-cast LiDAR, the driver and
  FAST-LIO interfaces, people dropped in at runtime, stand-in inspection), and an end-to-end CI
  test: patrol through a stop, stop for a person, drive on, the handheld cannot drive into a
  person, clean shutdown.

### Changed
- **Breaking:** `pashupati_bringup` → `pashupati` (`ros2 launch pashupati robot.launch.py`),
  `robot_mapping` merged into `robot_odometry`, `mapping/*` services → `route/*`, mode `mapping`
  → `teach` (old name still accepted).

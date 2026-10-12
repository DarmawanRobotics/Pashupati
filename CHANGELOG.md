# Changelog

All notable changes to the Pashupati robot stack. Format: [Keep a Changelog](https://keepachangelog.com),
versions follow [SemVer](https://semver.org).

## [Unreleased]

### Fixed
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
- CI: build, unit + in-process ROS tests, lint (flake8, pep257, shellcheck, YAML) on every PR.

### Changed
- **Breaking:** `pashupati_bringup` → `pashupati` (`ros2 launch pashupati robot.launch.py`),
  `robot_mapping` merged into `robot_odometry`, `mapping/*` services → `route/*`, mode `mapping`
  → `teach` (old name still accepted).

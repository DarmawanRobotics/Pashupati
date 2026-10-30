# Changelog

All notable changes to the Pashupati robot stack. Format: [Keep a Changelog](https://keepachangelog.com),
versions follow [SemVer](https://semver.org).

## [Unreleased]

### Fixed

- Dharma pinned to `main 2026-10-30` (`pashupati.repos`): nodes stop cleanly on Ctrl-C (no
  `take_message` RuntimeError, no publication after `destroy_node`), `planner/go_to` waits up to
  `pose_wait_sec` (2 s) for the robot pose right after localization, dharma_rviz builds with
  GCC 15.
- The end-to-end patrol test waits for the ARRIVED of its own goal: a late ARRIVED of the
  previous goal on a busy machine marked a stop point at the wrong place and the patrol then
  inspected only two of the three.

## [3.0.0] - 2026-10-29

The same patrol robot on Dharma 0.2 (`main 2026-10-28`, pinned in `pashupati.repos`).

### Changed

- **Breaking:** the Dharma 0.2 interfaces: `control_mux_node` is the only publisher of
  `/<robot_ns>/cmd_vel`; the handheld drives through `control/remote/handheld/cmd_vel`;
  services are `control/set_mode` (mode and source), `control/estop/engage|release`,
  `navigation/start|stop|pause|resume`, `navigation/load_path` (`path` inside the sites dir),
  `route/start|stop|mark_stop_point`, `planner/go_to`; status topics are typed messages
  (`ControlStatus`, `NavigationStatus`, `LocalizationStatus`, `HealthStatus`, `SiteStatus`,
  `StopPointEvent`, `DockingStatus`). The old `bridge/*` services and the string status topics
  are gone.
- **Breaking:** sites replace `map/`: a site holds its occupancy map, tags and taught routes
  under `SITES_DIR` (`~/.pashupati/sites`). `ROUTE` and `TAGS` are gone from `pashupati.env`;
  `SITE` and `SITES_DIR` replace them. `mode:=teach` maps the site from the start.
- **Breaking:** `ROBOT_BRINGUP` is `ROBOT_DRIVER`; the robot's footprint, speed limits and base
  height are the robot profile (`robot.*`, `src/pashupati/config/robot.yaml`, `ROBOT_PARAMS`)
  instead of `field.*` and per-node limits.
- **Breaking:** the configs are `config/robot.yaml` and `config/stack.yaml` (one file for the
  whole stack, the values tuned on the robot carried over) and `bridge.yaml`, `inspection.yaml`,
  `fleet.yaml` in their packages; the old `*_params.yaml` names are gone. A site file keyed by
  node name still goes on top of everything.
- **Breaking:** `script/` is `scripts/`.
- Every Pashupati node is a `DharmaNode` with a typed config (`declare_config`), a pure `core/`
  and its ROS helpers in `utils/`; lint is the hub's (ruff, prettier, clang-format, pre-commit).
- The inspection after a stop point is Dharma's follower hook (`hooks.inspection`); a disabled
  inspection answers at once and never holds the patrol. The back-up after an inspection is
  `inspection.reverse_distance_m` (0: off).
- Handheld protocol v1 and the NETRA API v1 keep their wire format; the differences are listed
  at the top of [PROTOCOL.md](src/pashupati_bridge/PROTOCOL.md) and
  [API.md](src/pashupati_fleet/API.md) (`TF_UNAVAILABLE` and `AVOIDANCE_STALE` alerts are now in
  `nav.message`).

### Added

- `pashupati_utils`: typed status to protocol names, health figures, launch helpers.
- Camera source: `CAMERA=rtsp` and `CAMERA_URL` start `rtsp_camera_node` for an IP camera.
- Docking: `dock` and `undock` from the handheld and NETRA (Dharma's `docking_node`, the robot
  driver's dock services).
- Patrol goals with the planner (`planner/go_to`) on the site's map.
- Dev container from the hub template (Humble, VNC): `compose.yaml`, `.devcontainer/`,
  `scripts/dev.sh`, `scripts/start_tmux.sh robot|sim`.
- `fake_netra` (`ros2 run pashupati_fleet fake_netra`): a local NETRA for the simulator and tests.
- End-to-end test on the simulator: localize, teach a route with three stop points, patrol it
  with the VLM inspection (stand-in Ollama), hand over to the handheld and back, report to NETRA.
- CI: pre-commit lint and the whole stack from `pashupati.repos` on Humble.

### Removed

- `docker-compose.yaml` and `docker/README.md` (now `compose.yaml` and the README).
- `map/example` (routes and tags belong to a site now).

## [2.0.0] - 2026-10-18

Same patrol as 1.0.0, now on the Dharma stack.

### Changed

- **Breaking:** odometry, localization, navigation, LiDAR perception, the control mux, the health
  monitor, the RViz panels and the simulator come from the `dharma_*` packages, pinned in
  `pashupati.repos` (`script/setup.sh install dharma`, or `vcs import src < pashupati.repos`).
- **Breaking:** the remaining packages are `pashupati`, `pashupati_bridge`, `pashupati_fleet` and
  `pashupati_inspection`; person detection is part of `pashupati_inspection`.
- The configs tuned on the robot live in `src/pashupati/config`; the simulation is
  `ros2 launch pashupati sim.launch.py` and its end-to-end test lives in `pashupati`.

### Added

- `script/setup.sh` component `dharma`.

## [1.0.0] - 2026-10-18

First stable release, field-tested at Lippo Mall Puri on a Genisom L1W.

### Added

- Handheld video over WebRTC: `robot.launch.py` starts MediaMTX (`script/setup.sh install
webrtc`), which runs the new `video_stream_node` while the handheld watches. H.264 from the
  Jetson hardware encoder (x264 or OpenH264 where there is none), read with the teleop token. The
  `welcome` offers it as `video: {port, path}` (WHEP); apps without WebRTC and robots without
  MediaMTX keep the JPEG frames.
- The handheld video shows the AprilTags the detector sees: `state.tags` carries their corners as
  fractions of the image (protocol v1, an extra field older apps ignore).
- Person boxes on the handheld video: `perception/people_status` and `state.people.boxes` carry
  them as fractions of the image. `rate_hz: 0` (new default) detects 5 times a second on the GPU,
  2 on the CPU.

- Own finding types from the handheld: add a type with a description of what to look for,
  choose whether it is reported, edit or remove it (`categories`, `alarm_categories` and the new
  `category_descriptions` are live and kept across restarts). Intervals below a second.
- Interval inspection queues its frames: every `interval_sec` a frame is taken even while the
  model is busy and waits in a first-in-first-out queue (kept across restarts, `queue_max` 50)
  that the model works through when free. Findings keep the time and pose of the frame.
- Guides in `docs/`, as A5 PDFs for reading on a phone: a User Manual for operators (Genisom L1W
  with the Pashupati Controller, real app screenshots) and a Tuning Guide for technicians (every
  parameter, the site file, measuring the braking, symptoms and fixes, and a parameter reference
  generated from the config files). `docs/pdf/build.py` rebuilds them; CI builds them too.

### Changed

- REMOTE drives without the protective field (`field_in_remote: false`): the operator watches the
  robot; the software e-stop and the speed envelope still apply. The simulator keeps it on.

### Fixed

- The robot ran out of RAM with inspection on (15.1 of 15.3 GB used, swapping): Ollama's
  llama-server kept up to 8 GiB of earlier prompts in RAM, useless when every prompt has a new
  image. `script/setup.sh install ollama` turns that cache off for the Ollama service. The
  detector also unloads its model when switched off or stopped instead of keeping it forever.
- The AprilTag detector got no camera_info from `apriltag_throttle` (best effort, the detector
  reads it reliable), so no tag was ever detected: the throttle publishes reliable.
- `ros2 topic list` stayed empty in a robot shell after `script/env.sh`: a ros2 daemon started
  from another environment kept its view; `env.sh` restarts it.
- Teleop camera stayed black: driver JPEGs (quality 95) were too big to arrive whole as one burst
  of datagrams. Frames above `camera_max_bytes` are re-encoded at `camera_jpeg_quality` and
  `camera_max_width`, the send buffer is larger, and a frame stops at its first dropped chunk.
- Lighter on the robot: the AprilTag detector gets the camera at 5 Hz (`apriltag_throttle`)
  instead of every frame; `lidar_sector_node` stops listening to TF once it knows the LiDAR
  mount, `route_recorder_node` only listens while recording, and `fleet_uplink_node` only runs
  when `CC_URL` is set. The health monitor watches the camera on `camera_info`: its subscription
  to the JPEG topic made the camera driver encode every frame all the time.
- `lidar_sector_node` with `input_type: livox` ran at ~6 Hz on a full Jetson core, because rclpy
  built a Python object for each of the ~20k points of a scan. It now reads the serialized
  CustomMsg straight into numpy (~1 ms per scan) and drops points with no return.
- Python nodes burned whole cores (`localization_node` 200 %): pip numpy's OpenBLAS threads
  busy-wait. One BLAS thread per process (`OPENBLAS_NUM_THREADS=1` in `run_robot.sh`, the env file
  and `script/env.sh`); TF listeners run on the node's own executor.
- Teleop: a handheld with a wrong token got no answer at all, now `denied`. Acks are per session:
  an app restarted with command ids from 1 got the previous session's ack and the command never
  ran. Camera, cloud and TF are subscribed only while a client wants them, and a reader thread
  replaces the 100 Hz polling timer (~1/3 of a core while nobody was connected).
- Fleet uplink and teleop crashed on a driver `DiagnosticStatus` (rclpy gives `level` as bytes).
- Obstacle markers in RViz blinked with "No transform": they are drawn on the latest TF.
- `run_robot.sh` passed empty launch arguments (`route:=`), which `ros2 launch` rejects;
  `robot.launch.py` reads them from the environment. `start_tmux.sh` passed empty `server_url:=` /
  `token:=` and did not use the service's RMW and domain.
- The service unit started Ollama even with `INSPECTION=false`; re-run
  `script/setup.sh install service`.
- Inspection retried silently when the model was missing; it now logs `ollama pull <model>`, or
  that Ollama is unreachable.
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

- `script/env.sh`: the service's environment in a shell (same RMW, domain and robot settings), and
  `pnode` to run one node with the package defaults and the site file.
- `script/setup.sh` reads `/etc/pashupati/pashupati.env` itself and has a `config` component that
  creates the env and site files, adds keys that are new in the example, and flags unquoted values
  with spaces. `check` reports an outdated service unit and a build older than the checkout, skips
  Ollama and the person model when they are disabled, and (Jetson) blacklists `uvcvideo` so the
  RealSense is not bound twice. `BUILD_WORKERS` / `BUILD_JOBS` limit the build.
- Teleop `welcome` carries the robot `name` and its `robot_commands` (the handheld's action menu).
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
- `robot_sim`: the production stack on a simulated quadruped (2D world, ray-cast LiDAR, the driver and
  FAST-LIO interfaces, people dropped in at runtime, stand-in inspection), and an end-to-end CI
  test: patrol through a stop, stop for a person, drive on, the handheld cannot drive into a
  person, clean shutdown.

### Changed

- Person detection and VLM inspection are off in the env example (`PEOPLE=false`,
  `INSPECTION=false`), which also sets `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` and
  `ROS_LOCALHOST_ONLY=1` (nothing on the robot's Wi-Fi can publish to the driver; 0 for RViz on
  a laptop or a driver on another computer).
- The RealSense driver comes from apt (`ros-humble-realsense2-camera`) instead of the
  `realsense-ros` submodule, which is removed. `script/setup.sh install realsense` installs it and
  the udev rules (from the package when it ships them, so it also works offline); the component
  replaces `realsense_udev`.
- **Following uses x, y and yaw.** The new default `holonomic` controller does three things:
  - It walks toward a carrot on the route in any direction of the body: cross-track errors close
    sideways without weaving, and the robot backs up or strafes when the route is behind or beside
    it (after a stop, when starting off the route).
  - It keeps its heading along the route.
  - It caps sideways and reverse speed.
- **Obstacles: path shift.** Obstacles on the route are passed by sliding sideways, within
  `path_shift.max_offset` (0.8 m) of the route and on the 360° safety scan. The robot returns
  once the obstacle is past its body, and stops short when there is no gap.
- Reactive avoidance (`avoidance_mode: reactive`, other controllers) now reacts within 1.0 m
  instead of 1.5 m.
- **Breaking:** `pashupati_bringup` → `pashupati` (`ros2 launch pashupati robot.launch.py`),
  `robot_mapping` merged into `robot_odometry`, `mapping/*` services → `route/*`, mode `mapping`
  → `teach` (old name still accepted).
- **Breaking:** robot-agnostic. Pashupati no longer ships a robot driver: any quadruped plugs in
  through a robot package (`ROBOT_BRINGUP` / `robot_bringup:=<package>[/<file>.launch.py]`,
  started with `namespace:=<robot_ns>`), whose contract is in `src/xtras/README.md`. The
  `genisom_l1w_ros2` submodule is removed, `sensors:=` is replaced by `robot_bringup:=`, the
  default `robot_ns` is `robot` (set `ROBOT_NS` to keep another one), the handheld's default
  robot commands are `stand_up`, `sit_down`, `emergency_stop` (extras via `robot_commands`), the
  vendor-autostart stop became `ROBOT_PRESTART`, and `script/setup.sh install robot` runs the
  robot package's own installer (`ROBOT_SETUP`). Health topics may use `{robot_ns}`.

<div align="center">

# PASHUPATI

**Autonomous patrol & inspection stack for quadruped robots (legged or wheel-legged, any brand with a ROS 2 driver) — LiDAR-inertial teach-and-repeat navigation that walks forward, sideways or backwards, path-shift obstacle avoidance, on-robot VLM inspection, crowd sensing, handheld teleop and the NETRA command center.**

[![CI](https://img.shields.io/github/actions/workflow/status/DarmawanRobotics/Pashupati/ci.yml?branch=dev&style=for-the-badge&label=CI)](https://github.com/DarmawanRobotics/Pashupati/actions/workflows/ci.yml)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/22.04/)
[![Python](https://img.shields.io/badge/Python-3.10-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://www.python.org/)
[![License: Apache-2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg?style=for-the-badge)](LICENSE)

</div>

---

## What it does

Mall corridors are patrolled on fixed routes every shift, so Pashupati uses **teach and repeat**
instead of a global planner:

- **Teach** — drive the robot once (handheld or RViz) while FAST-LIO2 tracks it, mark inspection
  stops, record the AprilTags that anchor the map. The route is uploaded to NETRA automatically.
- **Repeat** — routes are resampled and smoothed, then followed with a curvature-aware speed
  profile.
  - The default controller uses all three axes of a legged robot: it closes errors sideways, backs
    up or strafes when the route is beside or behind it, and keeps its heading along the route.
  - Obstacles are passed by sliding sideways within a bounded offset from the route (path shift);
    with no gap, the robot stops short.
  - At every stop the robot converges to 5 cm / 3° and inspects. A 360° protective field caps every
  command at a speed the robot can still stop at before the nearest obstacle in its path.
- **Inspect** — Moondream (local VLM via Ollama) checks each stop for trash, spills, floor damage
  or a fallen person; findings go to NETRA for final detection, validation and Telegram alerts.
- **Sense crowds** — people seen by the camera are placed on the floor plan and aggregated into
  per-minute crowd heatmaps.
- **Operate** — schedules and live control from NETRA, manual control from a Retroid handheld
  over UDP, one supervised systemd service on the robot with self-respawning nodes.

## System

```mermaid
flowchart LR
    subgraph Robot[Robot — Jetson, ROS 2 Humble]
        DRV[robot package<br/>driver · URDF] --- SENS[Livox MID-360 + RealSense]
        SENS --> FL[FAST-LIO2 / Leg-KILO] --> LOC[localization]
        SENS --> PER[lidar sectors · AprilTag · people]
        LOC --> NAV[path follower + avoidance]
        PER --> NAV
        NAV --> MUX[control mux] --> DRV
        INS[VLM inspection<br/>Moondream] --> UP[fleet uplink]
        NAV --> UP
        PER --> UP
        HL[health] --> UP
        TEL[teleop UDP gateway] --> MUX
    end
    PAD[Retroid teleop<br/>flutter_teleop] <-- UDP --> TEL
    UP <-- WebSocket + HTTPS --> NETRA[NETRA command center<br/>Go · PostgreSQL · Next.js]
    NETRA --> TG[Telegram groups]
```

| Repository | |
|---|---|
| **Pashupati** (this) | robot workspace |
| robot package | the robot's driver, URDF and sensor launch, added under `src/xtras/robots/` ([contract](src/xtras/README.md)) |
| [`leg_kilo`](src/xtras/slam/leg_kilo) | Leg-KILO odometry, `pashupati` branch of the fork: headless options (submodule, optional) |
| `netra` | command center: backend, dashboard, VPS deployment |
| `flutter_teleop` | handheld teleop app (Retroid Pocket / Android) |

## Hardware

| Component | Model | Role |
|---|---|---|
| Robot base | any quadruped (legged or wheel-legged) with a ROS 2 driver | robot package, [contract](src/xtras/README.md) |
| LiDAR | Livox **MID-360** | FAST-LIO2 odometry, obstacle sectors, teleop lidar view |
| Camera | Intel RealSense **D435i** | AprilTags, VLM inspection, people, teleop video |
| Compute | NVIDIA Jetson (aarch64) | ROS 2 Humble, Ollama, TensorRT |

## Packages

| Package | Role |
|---|---|
| [`robot_interfaces`](src/robot_interfaces) | messages and services |
| [`robot_odometry`](src/robot_odometry) | LiDAR-inertial odometry (FAST-LIO2 or Leg-KILO), stillness gate, odometry monitor, route recording |
| [`robot_localization`](src/robot_localization) | FAST-LIO frame bridge, `map→odom` from AprilTags / RViz, tag recording |
| [`robot_perception`](src/robot_perception) | LiDAR sectors, AprilTag, person detection |
| [`robot_navigation`](src/robot_navigation) | route processing, 5 controllers, 4 avoidance algorithms, final approach, safety |
| [`robot_bridge`](src/robot_bridge) | auto/remote control mux, UDP teleop gateway ([protocol](src/robot_bridge/PROTOCOL.md)) |
| [`robot_fleet`](src/robot_fleet) | NETRA uplink: telemetry, events, crowd, anomalies, routes ([API](src/robot_fleet/API.md)) |
| [`robot_inspection`](src/robot_inspection) | Moondream anomaly detection at stop points |
| [`robot_health`](src/robot_health) | node liveness, odometry / safety scan / e-stop state, sensor rates, computer health |
| [`robot_rviz`](src/robot_rviz) | RViz panels: Mission, Status, Tuning, AprilTags |
| [`pashupati`](src/pashupati) | the robot: one launch with respawning nodes, site configuration |
| [`robot_sim`](src/robot_sim) | simulated quadruped (driver, LiDAR, FAST-LIO interfaces) for the whole stack, end-to-end test |

## Install on the robot

Ubuntu 22.04 with ROS 2 Humble (JetPack 6 on a Jetson), internet, and the robot's SSH key on
GitHub.

```bash
git clone --recursive git@github.com:DarmawanRobotics/Pashupati.git ~/dev/Pashupati
cd ~/dev/Pashupati
git clone <robot package repo> src/xtras/robots/<name>   # driver + URDF + sensors, src/xtras/README.md
script/setup.sh install config          # creates /etc/pashupati/pashupati.env and site.yaml
sudoedit /etc/pashupati/pashupati.env   # ROBOT_BRINGUP, ROBOT_NS, ROBOT_SETUP, ROBOT_PRESTART, ROUTE, TAGS
sudoedit /etc/pashupati/site.yaml       # this robot: footprint, speeds, teleop token, robot_commands
script/setup.sh install                 # everything missing, then the build
script/setup.sh install service         # systemd unit, enabled at boot
sudo systemctl start pashupati && journalctl -fu pashupati
script/setup.sh                         # every line ok?
```

- **`pashupati.env`** holds the deployment (robot package, namespace, mode, route, NETRA). systemd
  and bash both read it: quote a value with spaces (`ROBOT_PRESTART="robot-launch stop x"`) and
  never put a comment after a value. `script/setup.sh` reads it too, so `ROBOT_SETUP` (the robot
  package's installer) and `ROBOT_IP` work without exporting anything.
- **`site.yaml`** holds the parameters of this robot and site, on top of every package default
  ([example](src/pashupati/config/site.example.yaml)). Set the same teleop `token` in the handheld.
- **Vendor stacks.** Pashupati runs on `rmw_fastrtps_cpp`, domain 0 (`RMW_IMPLEMENTATION`,
  `ROS_DOMAIN_ID`). A vendor autostart that also drives the robot or its LiDAR is stopped by
  `ROBOT_PRESTART` before every start.
- **ROS stays on the robot** (`ROS_LOCALHOST_ONLY=1`): nothing on the robot's Wi-Fi can publish to
  the driver around the control mux. Set it to 0 when the driver runs on another computer, or for
  RViz on a laptop while debugging (then `ros2 daemon stop` in open shells).
- **Heavy extras** are off: `PEOPLE=true` needs a TensorRT model (`script/setup.sh install
  people_model` prints the Jetson steps), `INSPECTION=true` needs Ollama and the VLM
  (`script/setup.sh install ollama`, then `install service` again).

The service runs `ros2 launch pashupati robot.launch.py` (`script/run_robot.sh`) with
`Restart=always`; every Pashupati node respawns on its own, and the last loaded route is reloaded
after a restart.

**Update:** `git pull && git submodule update --init --recursive && script/setup.sh install build
config service && sudo systemctl restart pashupati`. `install build` rebuilds whenever the checkout
moved; `install config` adds env keys that are new in the example; `install service` rewrites an
outdated unit.

**A shell on the robot** needs the service's environment, or `ros2 topic list` shows nothing (or
the vendor's topics):

```bash
echo "alias pashu='source ~/dev/Pashupati/script/env.sh'" >> ~/.bashrc   # once
pashu
ros2 topic hz /perception/safety_scan
pnode robot_bridge teleop_udp_node bridge_params.yaml   # one node, as robot.launch.py runs it
```

### Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `-bash: stop: command not found` when loading the env | value with spaces not quoted | quote it; `script/setup.sh` shows the line |
| `ros2 topic list` empty or only vendor topics | shell on another RMW, domain or `ROS_LOCALHOST_ONLY` | `pashu` (`script/env.sh`, it also restarts the ros2 daemon) |
| RViz on a laptop sees no topics | `ROS_LOCALHOST_ONLY=1` | set 0 in the env file while debugging, restart the service |
| robot or LiDAR also driven by the vendor stack | vendor autostart | `ROBOT_PRESTART` |
| camera drops out, udev and journal flooded every second | kernel `uvcvideo` and librealsense both bind the camera | `script/setup.sh install realsense` (blacklists `uvcvideo` on Jetson), reboot |
| a Python node at 100-200 % CPU | numpy OpenBLAS threads busy-wait | `OPENBLAS_NUM_THREADS=1` (env file) |
| handheld: "wrong token" | app token differs from `teleop_udp_node.token` | same token in both |
| handheld connected, robot does not walk | mode AUTO, or the e-stop is latched | REMOTE on the handheld, release the e-stop |
| `model ... is not installed in Ollama` | model pulled into `~/.ollama`, not into the service | `script/setup.sh install ollama` |
| `malformed launch argument 'route:='` | empty launch argument | leave it out; `robot.launch.py` reads the env |
| build killed (out of memory) | too many parallel compile jobs | `BUILD_WORKERS=1 BUILD_JOBS=2 script/setup.sh install build` |

## Develop (macOS / Linux)

```bash
script/dev.sh up      # Docker: ROS 2 Humble + all deps + XFCE desktop over VNC
script/dev.sh build   # colcon build (symlink-install, Release)
script/dev.sh test
script/dev.sh vnc     # TigerVNC localhost:5901 or http://localhost:6080/vnc.html
```

On the robot during development: `script/start_tmux.sh teach|patrol` (one pane per part, RViz,
package default parameters; settings from the env file).

## Simulation

The whole stack also runs against a simulated quadruped ([robot_sim](src/robot_sim)), with no robot,
Gazebo or GPU:

```bash
ros2 launch robot_sim sim.launch.py autostart:=true rviz:=true
# a person steps onto the route
ros2 topic pub --once /sim/obstacles geometry_msgs/msg/PoseArray "{poses: [{position: {x: 2.0, y: -3.0}}]}"
```

The simulated robot localizes itself and patrols a loop with an inspection stop. It stops for the
person and drives on once they are gone. The handheld (token `sim`) and NETRA (`cc_url:=`) connect
to it as they would to a real robot. Use it to try parameters, routes and new code before they go
on the robot.

## Workflow

### Teach

1. Start in teach mode (`MODE=teach` in the env file, or `start_tmux.sh teach`). Keep the robot
   standing still until odometry starts (`odometry/status` leaves `initializing`).
2. Take the handheld, switch to **REMOTE** and drive the route; **Record** starts recording,
   **Stop point** marks inspection stops with a dwell time (or the RViz Mission panel, or
   `route/record` / `route/mark_stop_point`).
3. Stand still in view of each AprilTag and **Record Tag Poses** (AprilTags panel or
   `localization/record_tags`) — tags recorded in the same session anchor the same map frame.
4. Stop recording: the route is saved under `map/<date>/` and uploaded to NETRA. End near the start
   for a looping patrol.

### Repeat

- **Localize** from a tag (teleop, panel or `localization/start`); navigation refuses to start
  until `localization/status` reports a source.
- **Start** from NETRA (now or weekly schedule), the handheld (**Routes** → load, **Start**) or:

```bash
ros2 service call /navigation/load_path robot_interfaces/srv/LoadPath "{waypoints_file: '/home/robot/dev/Pashupati/map/<date>/<time>_path.csv'}"
ros2 service call /navigation/start_nav std_srvs/srv/SetBool "{data: true}"
```

- At every stop the robot converges, calls `/anomaly_detector_node/check_now`, dwells and moves on;
  anomalies, crowd windows and stop events flow to NETRA.

### Emergency stop

The software e-stop is latched in the control mux. Engage it from the handheld **STOP** (screen
or gamepad B), from NETRA, or with
`ros2 service call /bridge/estop std_srvs/srv/SetBool "{data: true}"`. While it is engaged, only
zero velocity reaches the robot and navigation is paused. Release is explicit: `estop_release`
from the handheld or NETRA, or `{data: false}`. After a release, navigation stays paused until an
operator resumes it.
The driver's own stop is `ros2 service call /<robot_ns>/emergency_stop std_srvs/srv/Trigger`.

## Patrol behaviour

| Feature | What it does | Key parameters |
|---|---|---|
| Route processing | dedup, resample, gradient smoothing between stop points; stops never move | `resample_spacing`, `smooth_weight_*` |
| Controllers | `holonomic` (default: x, y and yaw, strafes and reverses, heading kept along the route), `pure_pursuit`, `pid`, `mppi`, `lqr`, `stanley`, switchable live | `controller`, `<controller>.*` |
| Speed profile | curvature speed cap, braking into stops and the route end | `max_lateral_accel`, `approach_speed` |
| Avoidance | path shift (default): slide sideways around obstacles on the 360° safety scan, never more than `max_offset` from the route, stop short without a gap; or reactive `braitenberg`, `vfh`, `potential_field`, `follow_gap` | `avoidance_mode`, `path_shift.*`, `algorithm` |
| Final approach | x / y / yaw convergence (strafing) to 5 cm / 3° | `holonomic`, `approach.*` |
| Inspection | VLM check at each stop, waits for the result, then dwells | `inspection_services`, `inspection_timeout_sec` |
| Loop patrol | restarts closed routes, ends the lap on low battery | `loop_route`, `low_battery_percentage` |
| Safety | stops on stale avoidance / odometry, e-stop hysteresis, `BLOCKED`, `OFF_PATH`, teleop watchdog | `*_timeout_sec`, `off_path_*` |
| Protective field | speed and separation monitoring in the mux for navigation and the handheld, in every direction: v·t + v²/2a ≤ clearance − margin; no fresh 360° scan, no autonomous motion | `field.*`, `safety_scan_timeout_sec` |
| Control mux | AUTO (navigation) or REMOTE (handheld); remote pauses navigation; zero when the active source goes silent; speed envelope, non-finite commands dropped | `bridge/set_mode`, `max_v*` |
| Software e-stop | latched in the mux: only zero reaches the robot until an explicit release, navigation stays paused | `bridge/estop` |
| Resilience | respawning nodes, supervised service, route remembered, uploads spooled offline | — |

Every navigation and avoidance parameter applies live (**Tuning** panel or `ros2 param set`); save
the tuned values with **Save YAML**.

## Operations

| | |
|---|---|
| Health | `ros2 topic echo /health/status` — `level` ok / warn / error; error for a missing node, lost odometry, a stale safety scan or an engaged e-stop; sensor rates, CPU temperature, memory, disk; also on the handheld and in NETRA |
| Logs | `journalctl -fu pashupati`; ROS logs older than 7 days are pruned at start |
| Debug view | RViz `robot_rviz/rviz/robot.rviz`: route coloured by speed, lookahead, cross-track, avoidance cone, approach target, tags |
| Update | `git pull && git submodule update --init --recursive && script/setup.sh install build config service && sudo systemctl restart pashupati` |

## Interfaces

<details>
<summary><b>Services</b></summary>

| Service | Type | Node |
|---|---|---|
| `/route/record` | `std_srvs/SetBool` | route_recorder_node |
| `/route/mark_stop_point` | `robot_interfaces/MarkStopPoint` | route_recorder_node |
| `/localization/start`, `/localization/record_tags` | `std_srvs/Trigger` | localization_node |
| `/navigation/load_path` | `robot_interfaces/LoadPath` | path_loader_node |
| `/navigation/start_nav`, `/navigation/pause` | `std_srvs/SetBool` | path_follower_node |
| `/bridge/set_mode` | `robot_interfaces/SetMode` | control_mux_node |
| `/bridge/estop` | `std_srvs/SetBool` (`true` engages, `false` releases) | control_mux_node |
| `/anomaly_detector_node/check_now` | `std_srvs/Trigger` | anomaly_detector_node |
| `/<robot_ns>/<command>` | `std_srvs/Trigger` | robot driver ([contract](src/xtras/README.md)) |

</details>

<details>
<summary><b>Topics</b></summary>

| Topic | Type | Publisher |
|---|---|---|
| `/<robot_ns>/cmd_vel` | `geometry_msgs/Twist` | control_mux_node |
| `/bridge/nav_cmd_vel`, `/bridge/remote_cmd_vel` | `geometry_msgs/Twist` | path_follower_node, teleop_udp_node |
| `/bridge/mode` | `std_msgs/String` (latched) | control_mux_node |
| `/bridge/estop_engaged`, `/bridge/estop_reason` | `std_msgs/Bool`, `std_msgs/String` (latched) | control_mux_node |
| `/bridge/estop_request` | `std_msgs/String` (reason) | any node: engages the e-stop |
| `/bridge/protective_field` | `std_msgs/String` (latched): `clear`, `limited`, `stop`, `stale`, `off` | control_mux_node |
| `/odometry/status` | `std_msgs/String` (latched) | odometry_monitor_node |
| `/odometry/restarted` | `std_msgs/Empty` | odometry_monitor_node |
| `/localization/status` | `std_msgs/String` (latched) | localization_node |
| `/navigation/waypoints` | `robot_interfaces/WaypointPath` (latched) | path_loader_node |
| `/navigation/status`, `/navigation/mission_status`, `/navigation/stop_point_event` | robot_interfaces | path_follower_node |
| `/navigation/markers`, `/navigation/route_markers`, `/navigation/avoidance_markers` | `MarkerArray` | navigation |
| `/perception/sector_scan` | `robot_interfaces/SectorScan` | lidar_sector_node |
| `/perception/safety_scan` | `robot_interfaces/SectorScan` (360°, all blocked when the LiDAR is blind) | lidar_sector_node |
| `/perception/people` | `geometry_msgs/PoseArray` | person_detector_node |
| `/anomaly_detector/result` | `std_msgs/String` (JSON) | anomaly_detector_node |
| `/health/status` | `std_msgs/String` (JSON, latched) | health_monitor_node |
| `/route/saved` | `std_msgs/String` | route_recorder_node |

</details>

<details>
<summary><b>TF tree</b></summary>

```
map → odom → camera_init → body → base_link ─┬─ livox_frame → livox_imu_frame
 (localization_node)  (FAST-LIO2)  (bridge)   └─ camera_link → camera_*_optical_frame
```

`camera_init → body` comes from FAST-LIO2 or Leg-KILO (`odometry:=leg_kilo`), same frame names.

</details>

## File formats

**Route CSV** — `x, y, yaw_deg, dwell_sec` in the `map` frame; `dwell_sec > 0` is an inspection stop.

```csv
# x,y,yaw_deg,dwell_sec
0.0,0.0,0,0
2.0,0.5,45,10.0
```

**Tag config JSON** — map pose per AprilTag TF frame (written by `localization/record_tags`).

```json
{ "base_map": { "x": 0.0, "y": 0.0, "z": 0.8, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0 } }
```

## Tests

```bash
colcon test --executor sequential && colcon test-result --verbose
```

Every package runs flake8 and pep257 plus its own tests. Navigation simulations drive every
controller around a noisy loop and an out-and-back corridor; bridge, fleet, inspection, health and
perception have unit tests for their protocol and logic; in-process ROS tests drive the real teleop
gateway over UDP, the odometry gate / monitor / route recorder with fake sensors, the path follower
state changes and the launch description. An end-to-end test runs the whole stack in the
simulator. The robot slides around a person on the route and still does the stop, stops for a
crowd and drives on once they leave, and cannot be driven into a person from the handheld. The
stack must shut down without a single dead node or traceback. CI runs all of it on every pull request
([.github/workflows/ci.yml](.github/workflows/ci.yml)); branch and commit rules are in
[CONTRIBUTING.md](CONTRIBUTING.md), changes in [CHANGELOG.md](CHANGELOG.md).

## Repository layout

```
Pashupati/
├── .github/workflows/ci.yml      # build + tests + lint on every PR
├── docker/ docker-compose.yaml   # dev container (VNC desktop, shared workspace)
├── map/                          # recorded routes + example/
├── script/                       # setup.sh, run_robot.sh (service), env.sh (shell), start_tmux.sh (dev), dev.sh
└── src/
    ├── robot_* pashupati         # first-party packages
    └── xtras/                    # submodules: livox_ros_driver2, FAST_LIO, Leg-KILO; robots/<your robot>
```

## License

[Apache License 2.0](LICENSE)

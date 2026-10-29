# pashupati

The robot package, modeled on the Dharma hub's `robot_bringup`. `robot.launch.py` starts the
whole stack in one process tree, the way it runs in production under systemd
(`scripts/setup.sh install service`); `sim.launch.py` starts the same stack on
[dharma_sim](https://github.com/DarmawanRobotics/dharma_sim).

```bash
ros2 launch pashupati robot.launch.py                        # patrol on the last site
ros2 launch pashupati robot.launch.py mode:=teach site:=mall # map the site `mall` from the start
ros2 launch pashupati robot.launch.py odometry:=leg_kilo     # Leg-KILO instead of FAST-LIO
ros2 launch pashupati robot.launch.py camera:=rtsp camera_url:=rtsp://10.0.0.20/stream
ros2 launch pashupati robot.launch.py robot_driver:=my_dog_bringup robot_ns:=dog
ros2 launch pashupati sim.launch.py rviz:=true               # the simulated robot
```

The stack is robot-agnostic. The robot's driver, URDF and sensors come from a robot package
started with `robot_driver`; [src/xtras/README.md](../xtras/README.md) lists what it must
provide.

## Configuration

| File                                       | What                                                                            |
| ------------------------------------------ | ------------------------------------------------------------------------------- |
| [config/robot.yaml](config/robot.yaml)     | the robot profile (`robot.*`): omni base, base height, footprint, limits        |
| [config/stack.yaml](config/stack.yaml)     | the Dharma stack and the Pashupati nodes as tuned on the robot (field tests)    |
| [config/sim.yaml](config/sim.yaml)         | what the simulator changes on top of `stack.yaml`                               |
| `/etc/pashupati/site.yaml` (`SITE_CONFIG`) | this robot and site, on top of everything ([example](config/site.example.yaml)) |
| `/etc/pashupati/pashupati.env`             | the launch arguments below, read by the service and `scripts/env.sh`            |

Every node gets the robot profile, then `stack.yaml`, then the site file, then the launch
arguments. A broken file stops the launch; a section that matches no node (a typo) is reported in
the log. A missing site file means no overrides.

| Argument                          | Default (env)                                |                                                                      |
| --------------------------------- | -------------------------------------------- | -------------------------------------------------------------------- |
| `robot_params`                    | `$ROBOT_PARAMS`, empty: `config/robot.yaml`  | robot profile                                                        |
| `params_file`                     | `$SITE_CONFIG`, `/etc/pashupati/site.yaml`   | site file                                                            |
| `robot_ns`                        | `$ROBOT_NS`, `robot`                         | driver namespace (`/robot/cmd_vel`, `/robot/odom`, ...)              |
| `robot_driver`                    | `$ROBOT_DRIVER`                              | `<package>[/<file>.launch.py]`; empty: the driver runs elsewhere     |
| `odometry`                        | `$ODOMETRY`, `fast_lio`                      | `fast_lio`, `leg_kilo` or `external`                                 |
| `mode`                            | `$MODE`, `patrol`                            | `teach` maps `site` from the start                                   |
| `site`, `sites_dir`               | `$SITE`, `$SITES_DIR` (`~/.pashupati/sites`) | the site (map, tags, taught routes); empty site: the last one        |
| `fiducials`, `fiducials_detector` | `true`, `$FIDUCIALS` (`apriltag`)            | camera throttle + tag detector (`apriltag` or `aruco`)               |
| `camera`, `camera_url`            | `$CAMERA` (`driver`), `$CAMERA_URL`          | `driver`: the robot package publishes it; `rtsp`: `rtsp_camera_node` |
| `camera_topics`                   | `$CAMERA_TOPICS`, `/camera/camera/color`     | prefix of `image_raw` and `camera_info`                              |
| `inspection`, `ollama_url`        | `$INSPECTION`, `$OLLAMA_URL`                 | VLM inspection (Ollama)                                              |
| `people`                          | `$PEOPLE`                                    | person detection                                                     |
| `webrtc`, `mediamtx`              | `$WEBRTC` (`true`), `$MEDIAMTX`              | handheld video over WebRTC through MediaMTX; off: JPEG frames        |
| `cc_url`, `cc_token`, `robot_id`  | `$CC_URL`, `$CC_TOKEN`, `$ROBOT_ID`          | NETRA command center; empty url: no uplink                           |
| `health`, `log_level`             | `true`, `info`                               |                                                                      |

`sim.launch.py` takes the same arguments minus the driver, odometry, video and site file ones,
plus `world`, `scenario` and `rviz`; it defaults to the `mall_loop` world, sites in
`/tmp/pashupati_sim/sites`, the handheld token `sim` and no fiducials, inspection or people.

Invalid values (`mode:=auto`, `odometry:=gmapping`, `robot_ns:=dog-2`, a robot package that is
not built) stop the launch with a clear error instead of starting a half-configured robot. The
first log line states the version, git commit (`PASHUPATI_GIT`, set by `scripts/run_robot.sh`),
robot id, mode, odometry and site file.

## Started nodes

The robot package (`robot_driver`); odometry (FAST-LIO held until the robot is still, odometry
monitor); localization (tags, `map -> odom`); LiDAR sectors and the 360° safety scan; tag
detector; site manager, grid mapper, route recorder; planner, path loader, path follower,
obstacle avoidance, docking; the control mux; health monitor; the handheld gateway
(`teleop_udp_node`, MediaMTX with `video_stream_node` while the handheld watches, `rtsp_camera`
with `camera:=rtsp`); the anomaly and person detectors; the NETRA uplink. Every node respawns 3 s
after it dies, and the service restarts the whole launch if it exits.

## WebRTC video

MediaMTX v1.11 or newer serves the handheld's video: WHEP on TCP 8890, media on UDP 8190, and the
encoder publishes to it over RTSP on `127.0.0.1:8654`. The launch writes its configuration to
`~/.pashupati/mediamtx.yml` (mode 600) at every start: reading needs the `teleop_udp_node` token
from the site file (user `teleop`), publishing is allowed from the robot itself only. Encoder
settings (`fps`, `max_width`, `bitrate_kbps`, `encoder`) are `video_stream_node` parameters.

## Tests

`test/unit` covers the launch helpers, `test/ros/test_launch.py` the launch files, and
`test/e2e/test_sim_patrol.py` the whole robot on the simulator: it localizes, a route with three
stop points is taught, the route is patrolled to its end with the VLM inspection answered at
every stop (a stand-in Ollama), the handheld takes over through the control mux and gives it
back, and NETRA (`fake_netra`) receives telemetry, stop events, findings and the route.
`PASHUPATI_SKIP_SIM=1` skips it.

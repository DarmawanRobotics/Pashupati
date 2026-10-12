# pashupati

The robot package. `robot.launch.py` starts the whole stack in one process tree, the way it runs in
production under systemd (`script/setup.sh install service`). Every Pashupati node respawns 3 s after
it dies, and the service restarts the whole launch if it exits.

```bash
ros2 launch pashupati robot.launch.py                       # patrol, last loaded route
ros2 launch pashupati robot.launch.py mode:=teach           # keep the map while teaching a route
ros2 launch pashupati robot.launch.py odometry:=leg_kilo    # Leg-KILO instead of FAST-LIO
ros2 launch pashupati robot.launch.py route:=/path/route.csv people:=false
```

| Argument | Default | |
|---|---|---|
| `robot_id` | `$ROBOT_ID` | fleet id reported to NETRA; empty: `fleet_params.yaml` / site file |
| `robot_ns` | `$ROBOT_NS` or `l1w` | robot driver namespace (`/l1w/cmd_vel`, `/l1w/odom`, ...) |
| `mode` | `$MODE` or `patrol` | `teach` keeps the map (FAST-LIO PCD / Leg-KILO backend, RAM grows); `mapping` still works |
| `odometry` | `$ODOMETRY` or `fast_lio` | `fast_lio`, `leg_kilo` or `external` (simulator) ([robot_odometry](../robot_odometry)) |
| `route` | `$ROUTE` | route CSV; empty reloads the last route loaded through `navigation/load_path` |
| `tags` | `$TAGS` | AprilTag map poses (JSON), created by `localization/record_tags` |
| `cc_url`, `cc_token` | `$CC_URL`, `$CC_TOKEN` | NETRA command center |
| `site_config` | `$SITE_CONFIG` or `/etc/pashupati/site.yaml` | per-robot parameter overrides, see below |
| `sensors`, `people`, `inspection`, `health` | `true` (`$PEOPLE`, `$INSPECTION`) | optional parts |

Invalid values (`mode:=auto`, `odometry:=gmapping`, `robot_ns:=l1w-2`) stop the launch with a clear
error instead of starting a half-configured robot. The first log line states the version, git
commit (`PASHUPATI_GIT`, set by `script/run_robot.sh`), robot id, mode, odometry and site file.

## Site configuration

Tuning for one mall or one robot belongs in `/etc/pashupati/site.yaml`, not in the repository. It
is an ordinary ROS 2 parameter file keyed by node name and is applied on top of the package
defaults of every node (including FAST-LIO and the odometry gate):

```yaml
path_follower_node:
  ros__parameters:
    target_linear_velocity: 0.35
teleop_udp_node:
  ros__parameters:
    token: "change-me"
```

A missing file means no overrides. A broken file, or a section without `ros__parameters`, stops
the launch, and a section that matches no node (a typo) is reported in the log. Non-empty launch
arguments (`ROUTE`, `TAGS`, `CC_URL`, ...) win over the site file. Start from
[config/site.example.yaml](config/site.example.yaml).

Started nodes: robot driver + Livox + RealSense (`l1w_bringup`), odometry (held until the robot is
still) + odometry monitor + route recorder, localization, lidar sectors, AprilTag, person detector,
path loader / follower / avoidance, control mux, teleop UDP gateway, NETRA uplink, VLM anomaly
detector, health monitor. `script/start_tmux.sh` remains for development (one pane per part, RViz).

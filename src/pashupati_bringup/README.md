# pashupati_bringup

`robot.launch.py` starts the complete robot in one process tree — the way it runs in production
under systemd (`script/setup.sh install service`). Every Pashupati node has `respawn` with a 3 s
delay, so a crashed node comes back on its own; the service restarts the whole launch if it exits.

```bash
ros2 launch pashupati_bringup robot.launch.py                       # patrol, last loaded route
ros2 launch pashupati_bringup robot.launch.py mode:=teach           # keep the map while teaching
ros2 launch pashupati_bringup robot.launch.py odometry:=leg_kilo     # Leg-KILO instead of FAST-LIO
ros2 launch pashupati_bringup robot.launch.py route:=/path/route.csv people:=false
```

| Argument | Default | |
|---|---|---|
| `robot_ns` | `$ROBOT_NS` or `l1w` | robot driver namespace |
| `mode` | `patrol` | `teach` keeps the map (FAST-LIO PCD / Leg-KILO backend, RAM grows); `mapping` still works |
| `odometry` | `$ODOMETRY` or `fast_lio` | `fast_lio` or `leg_kilo` ([robot_odometry](../robot_odometry)) |
| `route` | empty | route CSV; empty reloads the last route loaded through `navigation/load_path` |
| `tags` | `$TAGS` | AprilTag map poses (JSON) |
| `cc_url`, `cc_token` | `$CC_URL`, `$CC_TOKEN` | NETRA command center |
| `sensors`, `people`, `inspection`, `health` | `true` | optional parts |

Started nodes: robot driver + Livox + RealSense (`l1w_bringup`), odometry (FAST-LIO or Leg-KILO,
held until the robot is still) + odometry monitor + route recorder,
localization, lidar sectors, AprilTag, person detector, path loader / follower / avoidance,
control mux, teleop UDP gateway, NETRA uplink, VLM anomaly detector, health monitor.
`script/start_tmux.sh` remains for development (one pane per part, RViz).

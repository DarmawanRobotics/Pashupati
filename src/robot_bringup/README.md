# robot_bringup

Top-level entry point for the whole robot: one launch file, one RViz config,
one tmuxp session. Doesn't implement anything itself -- it just wires
together `robot_description`, `robot_drivers`, `robot_mapping`,
`robot_localization`, `robot_perception`, `robot_navigation`, and
`robot_mission`.

## Two ways to start the stack -- pick one, not both

Running `bringup.launch.py` and the tmux session at the same time starts
every node twice (port/service conflicts, double `cmd_vel` publishers, etc).

### 1. `bringup.launch.py` -- single command, unattended
```bash
ros2 launch robot_bringup bringup.launch.py
```
Args: `use_rviz` (default `true`), `waypoints_file` (default
`/home/robot/dev/Pashupati/map/example/example_waypoint.csv`), `tree_file`
(default `.../robot_mission/trees/patrol.xml`).

Startup is staged with `TimerAction` delays (description+drivers at t=0,
mapping/localization at t=3s, perception at t=4s, navigation at t=6s,
mission at t=7s) so each stage's dependencies (TF, Livox data, SectorScan,
the `navigate_route` action server) are more likely to already exist by the
time the next stage starts. **These are fixed delays, not real readiness
checks** -- on a slow boot (cold SDK connect, USB re-enumeration) a stage can
still start before what it depends on is actually up. It'll generally
recover on its own (TF lookups retry, action clients wait for the server),
just not instantly. If that's a problem in practice, the fix is to replace
the relevant `TimerAction` with an actual readiness signal (e.g. a
`RegisterEventHandler` on the driver connecting) rather than raising the
delay numbers further.

### 2. tmuxp session -- one visible window per subsystem
```bash
pip install --user tmuxp
tmuxp load src/robot_bringup/tmux/pashupati.yaml
```
Every subsystem gets its own window (`Ctrl-b` then a number to switch), so
you can watch or restart one piece without touching the rest, plus a
`control` window listing the manual service/action calls you'll actually
use during testing (localization trigger, path recording, stop-point
marking, sending a navigate_route goal). No staged timing here -- you start
each window whenever you're ready for that subsystem, which is the point of
using this over `bringup.launch.py` while developing.

## RViz

`rviz/robot.rviz`: RobotModel, TF, the loaded `Path`, `navigation/markers`
(lookahead/curvature/status), `perception/obstacles` (lidar sector fan), and
a color camera `Image` display (off by default). Fixed frame is `map`.
```bash
ros2 launch robot_bringup rviz.launch.py
```

## Known limitations
- `bringup.launch.py`'s stage timing is fixed delays, not health checks --
  see the note above.
- Neither launch path calls `localization/start` for you -- that's a
  deliberate manual step (face a configured tag, then trigger it), not
  something safe to automate blindly on every boot.
- The tmux `control` window only prints example commands, it doesn't run
  anything -- type/paste the one you need.

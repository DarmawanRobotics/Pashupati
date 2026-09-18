# robot_ui

Native RViz2 panel (C++/Qt) wrapping the operator-facing controls for
`robot_localization`, `robot_mapping`, and `robot_navigation` -- no more
typing `ros2 service call`/`ros2 action send_goal` by hand.

This is the first C++ package in an otherwise all-Python workspace, because
RViz2's `Panel` plugin architecture (`pluginlib`) is C++/Qt only -- there is
no supported pure-Python way to dock a panel inside RViz's own window as of
Humble. Written but **not compile-tested here** (no ROS2/Qt5 toolchain in
this sandbox) -- see "If it doesn't build" below.

## Layout

| Group | Controls | Talks to |
|---|---|---|
| Battery | color-coded progress bar (green/orange/red) | `drivers/battery` (`sensor_msgs/BatteryState`) |
| Localization | "Trigger Localization" button | `localization/start` (`std_srvs/Trigger`) |
| Path Recording | Start/Stop Recording, dwell spinbox + "Mark Stop Point" | `mapping/path_record` (`SetBool`), `mapping/mark_stop_point` (`MarkStopPoint`) |
| Path Loading | file path field + Browse + Load | `navigation/load_path` (`LoadPath`, new -- see below) |
| Mission (Behavior Tree) | root status + active behavior name, Pause/Resume button | `mission/status` topic, `mission/set_active` (`SetBool`, new -- see below) |
| Navigation | Start/Cancel, progress bar, color-coded status | `navigate_route` action, `navigation/status` topic |

**"Start Navigation" is disabled while the mission is active** (default on
startup, matching `mission_node`'s own default) -- pause the mission first
to free it up. This is enforced in the GUI itself now, not just a "don't run
both" rule to remember.

## New: `mission/status` + `mission/set_active` (robot_mission)

`mission_node` previously ran the tree with zero external visibility or
control. It now publishes `robot_interfaces/msg/MissionStatus` (`active`,
`root_status`, `active_behavior` -- the name of whichever leaf/decorator is
currently the tree's `tip()`) every tick, and exposes `mission/set_active`
(`std_srvs/SetBool`). Setting it `false` doesn't just stop ticking -- it
calls `tree.root.stop(Status.INVALID)`, which recursively terminates every
currently-RUNNING behaviour with `INVALID`, and `NavigateRouteAction`'s
existing `terminate()` already cancels its in-flight action goal on exactly
that transition. So pausing the mission actually frees the action server,
it doesn't just stop the tree from *trying* to use it.

The battery bar's red/orange threshold (20% / 40%) matches
`robot_driver_node`'s own low-battery gate for enabling auto mode (< 20% ==
"battery low", `set_auto_mode` will refuse) -- red on this bar means auto
mode won't turn on, not just "getting low".

**Note:** `robot_driver_node`'s `poll_state()` only publishes
`drivers/battery` while the SDK connection is bound (i.e. auto mode has
been enabled at least once) -- while purely in manual/unbound mode, this
bar just shows "no data yet" and never updates. That's existing driver
behavior, not something this panel controls.

## New: `navigation/load_path` service

`path_loader_node` previously only read `waypoints_file` once at launch
(a ROS param, not runtime-settable). The panel's "Load Path" button needed
a way to swap routes without relaunching, so `path_loader_node.py` gained:
```
robot_interfaces/srv/LoadPath
string waypoints_file
---
bool success
string message
```
Calling it re-reads the given CSV and republishes both `navigation/path`
and `navigation/waypoints` immediately. This is a real, needed addition to
`robot_navigation`, not something scoped to just this panel -- anything
else could call it too.

## How ROS and Qt coexist in one panel

RViz's event loop is Qt's (`QApplication::exec()`), not `rclcpp::spin()`.
The panel creates its own `rclcpp::Node` in `onInitialize()` and pumps it
with `rclcpp::spin_some()` on a 50ms `QTimer` (`onSpinRos()`) instead of a
real blocking spin, which would freeze the whole RViz GUI. All service/
action callbacks (button click -> response arriving) run through that timer
tick, not immediately -- expect up to ~50ms latency on status updates,
never a frozen UI.

## Build
```bash
sudo apt install qtbase5-dev ros-humble-rviz-common ros-humble-pluginlib
colcon build --packages-select robot_interfaces robot_ui
source install/setup.bash
```

## Add it to RViz
1. Launch RViz (`ros2 launch robot_bringup rviz.launch.py`, or plain `rviz2`).
2. Panels menu -> Add New Panel.
3. Find **robot_ui -> ControlPanel** in the list, select it, OK.
4. Dock it wherever -- RViz remembers panel layout in its own config next
   time you save it (File -> Save Config), independent of `robot.rviz`
   unless you save over that file.

## If it doesn't build
- `Qt5Config.cmake not found` -> `qtbase5-dev` isn't installed.
- `rviz_common/panel.hpp: No such file` -> `ros-humble-rviz-common` (dev
  headers) missing, or sourcing the wrong ROS distro.
- Undefined reference to `vtable for ControlPanel` / moc-related linker
  errors -> `CMAKE_AUTOMOC` didn't pick up the `Q_OBJECT` macro; make sure
  a clean rebuild (`rm -rf build/robot_ui install/robot_ui`)
  actually reruns CMake's configure step, not just make.
- Panel doesn't show up in "Add New Panel" -> `plugin_description.xml`
  isn't registered; check `ros2 pkg xml robot_ui` mentions
  `rviz_common__pluginlib__plugin` under exports, and that you sourced
  `install/setup.bash` from *this* build, not an old one.

## Known limitations
- Pausing the mission cancels its *current* goal, but nothing stops
  `mission_node` from being resumed (or the operator forgetting to pause it)
  while the panel also has a goal in flight -- the action server's one-goal-
  at-a-time rule still applies, this just removes the most common way to
  trip over it.
- No timeout/retry on the initial `service_is_ready()`/
  `wait_for_action_server()` checks -- if a server isn't up yet, the button
  just reports "not available" once; click again after it's up rather than
  expecting the panel to wait for you.
- "Load Path" and "Mark Stop Point" don't validate the CSV/dwell value
  beyond what the server side already does -- garbage in, whatever
  `path_loader_node`/`path_recorder_node` already do with garbage.

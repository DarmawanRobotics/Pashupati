# robot_bridge

Who drives the robot, and how the handheld talks to it without ROS.

## Nodes

### control_mux_node

The single gate between every velocity source and the robot: forwards either navigation or teleop
velocity, and holds the robot still while the software e-stop is engaged.

| | Name | Type |
|---|---|---|
| sub | `bridge/nav_cmd_vel` | `geometry_msgs/Twist` (path follower output) |
| sub | `bridge/remote_cmd_vel` | `geometry_msgs/Twist` (teleop) |
| sub | `bridge/estop_request` | `std_msgs/String`: engage the e-stop with this reason, no reply |
| pub | `cmd_vel` | `geometry_msgs/Twist` (remap to the robot, e.g. `/l1w/cmd_vel`) |
| pub | `bridge/mode` | `std_msgs/String`, latched |
| pub | `bridge/estop_engaged`, `bridge/estop_reason` | `std_msgs/Bool`, `std_msgs/String`, latched |
| srv | `bridge/set_mode` | `robot_interfaces/SetMode` (`auto` \| `remote`) |
| srv | `bridge/estop` | `std_srvs/SetBool`: `true` engages, `false` releases |

In `remote` mode navigation is paused (and resumed when switching back). The robot gets one zero
command as soon as the active source goes silent: teleop after `remote_timeout_sec`, navigation
after `nav_timeout_sec`.

**Software e-stop.** Engaged from the handheld (`estop`, and also its `emergency_stop` driver
command), NETRA (`estop`), RViz / CLI (`ros2 service call /bridge/estop std_srvs/srv/SetBool
"{data: true}"`) or any node via `bridge/estop_request`. While engaged nothing but zero reaches the
robot (sent at `estop_hold_hz`), navigation is paused and a mode switch cannot resume it. Release
is explicit (`estop_release`, or the service with `false`); navigation then stays paused until an
operator resumes it.

### teleop_udp_node

UDP gateway for the Flutter teleop: joystick, commands (mode, navigation, route recording, stop
points, routes, localization, robot commands), 10 Hz telemetry, JPEG camera, downsampled lidar and
the loaded route. The wire format is in [PROTOCOL.md](PROTOCOL.md).

## Run

```bash
ros2 launch robot_bridge bridge.launch.py robot_ns:=l1w
ros2 launch robot_navigation navigation.launch.py cmd_vel_topic:=/bridge/nav_cmd_vel ...
```

Parameters: [config/bridge_params.yaml](config/bridge_params.yaml). For a lighter camera stream
publish a compressed image (`camera_compressed: true`); raw images are JPEG-encoded with OpenCV.

## Test

```bash
colcon test --packages-select robot_bridge && colcon test-result --verbose
```

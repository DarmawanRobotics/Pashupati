# robot_bridge

Who drives the robot, and how the handheld talks to it without ROS.

## Nodes

### control_mux_node

Forwards either navigation or teleop velocity to the robot.

| | Name | Type |
|---|---|---|
| sub | `bridge/nav_cmd_vel` | `geometry_msgs/Twist` (path follower output) |
| sub | `bridge/remote_cmd_vel` | `geometry_msgs/Twist` (teleop) |
| pub | `cmd_vel` | `geometry_msgs/Twist` (remap to the robot, e.g. `/l1w/cmd_vel`) |
| pub | `bridge/mode` | `std_msgs/String`, latched |
| srv | `bridge/set_mode` | `robot_interfaces/SetMode` (`auto` \| `remote`) |

In `remote` mode navigation is paused (and resumed when switching back), and the robot gets a zero
command as soon as the teleop stops sending for `remote_timeout_sec`.

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

# pashupati_bridge

How the handheld talks to the robot without ROS, its video, and where the camera comes from.
The bridge never drives the robot itself: the stick is one operator source of Dharma's control
mux, which alone publishes the robot's `cmd_vel` after its envelope, protective field and e-stop.

| Node                | Role                                                                                    |
| ------------------- | --------------------------------------------------------------------------------------- |
| `teleop_udp_node`   | UDP gateway of the Pashupati Controller: stick, commands, telemetry, JPEG camera, cloud |
| `video_stream_node` | H.264 of the camera for the handheld's WebRTC video (MediaMTX starts it while watched)  |
| `rtsp_camera_node`  | an RTSP camera as the robot's camera (`camera:=rtsp`): image, JPEG and `camera_info`    |

## teleop_udp_node

The wire format is [PROTOCOL.md](PROTOCOL.md) (v1, unchanged for the handheld).

| Handheld                       | Dharma                                                                                                                      |
| ------------------------------ | --------------------------------------------------------------------------------------------------------------------------- |
| stick (`joy`)                  | `control/remote/handheld/cmd_vel` (`remote_source`), clamped to `max_vx/vy/wz` and `robot.limits`; NaN or text is 0         |
| `set_mode`                     | `control/set_mode` (`auto`, or `remote` with source `handheld`)                                                             |
| `estop`                        | `control/estop_request` at once, then `control/estop/engage`; `estop_release`: `control/estop/release`                      |
| `nav_*`                        | `navigation/start`, `stop`, `pause`, `resume`                                                                               |
| `record_*`                     | `route/start`, `route/stop`, `route/mark_stop_point` (dharma_mapping)                                                       |
| `list_routes`, `load_route`    | the taught routes of every site under `sites_dir`, `navigation/load_path`                                                   |
| `localize`, `record_tags`      | `localization/start`, `localization/record_tags`                                                                            |
| `dock`, `undock`               | `docking/dock`, `docking/undock`                                                                                            |
| `nav_speed`                    | `path_follower_node` `speed.cruise`, between `patrol_speed_min` and `patrol_speed_max` (never above `robot.limits.max_vx`)  |
| `inspection_set`, `people_set` | live parameters of `anomaly_detector_node`, `person_detector_node`                                                          |
| robot commands                 | `std_srvs/Trigger` `/<robot_ns>/<name>` of the driver (`robot_commands`; `emergency_stop` also latches the software e-stop) |

Telemetry comes from the typed status: `control/status` (mode, e-stop, field), `navigation/status`,
`localization/status`, `health/status` with the computer's figures from the health monitor's
`/diagnostics`, the detectors' status, `navigation/waypoints`, and the driver's `battery`,
`diagnostics` and `cmd_vel` (remapped to `/<robot_ns>/` by the launch).

Set the same `token` in the site file and in the handheld; a wrong one is answered with `denied`.
Camera, cloud and TF are subscribed only while a handheld is connected and wants them.

## video_stream_node

MediaMTX (the `pashupati` bringup, `scripts/setup.sh install webrtc`) runs it while the handheld
watches the WebRTC video and stops it 10 s after the last viewer. It encodes `image_topic` (raw
`rgb8`, `bgr8`, `mono8`, `yuv422`, ...) with the first encoder that works (Jetson
`nvv4l2h264enc`, x264, OpenH264) and publishes over RTSP to MediaMTX on the loopback.

## rtsp_camera_node

An RTSP camera (an IP camera, a gimbal pod, a robot's own camera) in place of a camera driver:
`camera:=rtsp camera_url:=rtsp://...` of the bringup starts it, remapped onto the camera topics
every consumer reads (`<camera_topics>/image_raw`, `image_raw/compressed`, `camera_info`). It
decodes with GStreamer (Jetson hardware decoder, libav, then FFmpeg), reconnects with a backoff
up to 30 s, and with `calibration_file` publishes the rectified image with a matching
`camera_info` ([config/calibration.example.yaml](config/calibration.example.yaml)); without one
`camera_info` assumes `default_hfov_deg` and tag poses are only approximate.

## Layout

| Module                                                 |                                                             |
| ------------------------------------------------------ | ----------------------------------------------------------- |
| `teleop_udp_node.py`                                   | messages, commands, the state message                       |
| `core/protocol.py`                                     | wire format: JSON, chunked streams, point packing           |
| `core/session.py`                                      | one driver at a time, acknowledgements, `denied` rate limit |
| `core/commands.py`                                     | which service each command calls, checks of the arguments   |
| `core/summaries.py`                                    | the inspection, people and health parts of `state`          |
| `core/routes.py`                                       | taught routes inside the sites directory                    |
| `core/config.py`                                       | `TeleopConfig`, `VideoConfig`, `CameraConfig`               |
| `core/video.py`, `core/rtsp.py`, `core/calibration.py` | GStreamer pipelines, the camera calibration                 |
| `utils/link.py`                                        | the UDP socket with its reader thread                       |
| `utils/clients.py`                                     | the service clients, the patrol speed                       |
| `utils/telemetry.py`                                   | the status subscriptions                                    |
| `utils/streams.py`                                     | camera, cloud and tags, subscribed only while wanted        |

## Launch and test

```bash
ros2 launch pashupati_bridge bridge.launch.py robot_ns:=robot
ros2 launch pashupati_bridge bridge.launch.py camera:=rtsp camera_url:=rtsp://192.168.1.120:8554/test
colcon test --packages-select pashupati_bridge && colcon test-result --verbose
```

Arguments: `params_file`, `robot_params`, `use_sim_time`, `log_level`, `robot_ns`, `teleop`,
`video_port`, `camera` (`driver` | `rtsp`), `camera_url`, `camera_topics`
(`/camera/camera/color`). Defaults: [config/bridge.yaml](config/bridge.yaml).

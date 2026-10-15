# robot_inspection

`anomaly_detector_node` — early anomaly detection on the robot with a local vision-language model
(Ollama, no cloud). The NETRA command center runs the final detection and the operator validation.

| Interface | Type | |
|---|---|---|
| `image_topic` (`/camera/camera/color/image_raw`) | `sensor_msgs/Image` | camera input |
| `~/check_now` (`/anomaly_detector_node/check_now`) | `std_srvs/Trigger` | inspect the latest frame now; called by `path_follower_node` at every stop point |
| `anomaly_detector/result` | `std_msgs/String` (JSON) | `{category, description, model, latency_ms, is_anomaly, source, evidence_stamp}`, published only for `alarm_categories`; `robot_fleet` uploads them with the evidence frame and pose |
| `anomaly_detector/evidence` | `sensor_msgs/CompressedImage` | the camera frame the result was detected in (full size), published just before the result; its stamp equals `evidence_stamp` |

The model picks one of `categories` (default `trash`, `spill`, `stain`) or `none`. Only
`alarm_categories` (default `trash`, `spill`) are reported; `stain` is too easily confused with
reflections on shiny floors. With `interval_sec: 0` the node inspects back to back and reports a
category at most once per `cooldown_sec`; `check_now` (source `stop_point`) always reports.

Debug parameters: `model`, `use_schema` (false = free-text `FLOOR:`/`TYPE:` answer, for models
whose runner crashes on grammar), `describe` (one sentence about the floor) and `debug` (log every
inspection and the raw answer).

## Run

```bash
ros2 launch robot_inspection inspection.launch.py
ros2 run robot_inspection anomaly_detector_node --ros-args \
  -p model:=ahmadwaqar/smolvlm2-2.2b-instruct:latest -p describe:=true -p debug:=true
ros2 service call /anomaly_detector_node/check_now std_srvs/srv/Trigger
ros2 topic echo /anomaly_detector/result
```

Requires Ollama >= 0.12.7 for `qwen3-vl`.

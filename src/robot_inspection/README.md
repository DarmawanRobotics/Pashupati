# robot_inspection

`anomaly_detector_node` — early anomaly detection on the robot with a local vision-language model
(Moondream through Ollama, no cloud). The NETRA command center runs the final detection and the
operator validation.

| Interface | Type | |
|---|---|---|
| `~/check_now` (`/anomaly_detector_node/check_now`) | `std_srvs/Trigger` | inspect the latest frame; called by `path_follower_node` at every stop point (`inspection_services`) |
| `anomaly_detector/result` | `std_msgs/String` (JSON) | `{is_anomaly, category, confidence, description, model, latency_ms, source}`, published only for anomalies above `min_confidence`; `robot_fleet` uploads them with the frame and pose |

The prompt asks for one of `categories` (default `trash`, `spill`, `floor_damage`,
`fallen_person`) as JSON; Ollama constrains the output to JSON and off-list answers are mapped by
keywords, so a small model stays usable. The model is loaded at start-up and kept in memory
(`keep_alive`), images are downscaled to `max_image_width`, and only one inference runs at a time.
With `periodic_check_sec > 0` the robot also inspects while driving, reporting a category at most
once per `cooldown_sec`.

## Setup

```bash
script/setup.sh install ollama          # installs Ollama and pulls moondream
ros2 launch robot_inspection inspection.launch.py
ros2 service call /anomaly_detector_node/check_now std_srvs/srv/Trigger
```

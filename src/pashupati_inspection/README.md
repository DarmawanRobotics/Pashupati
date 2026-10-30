# pashupati_inspection

What the robot looks at: early anomaly detection with a local vision-language model (Ollama, no
cloud) at the stop points of a patrol, and people counted on the camera and placed on the map.
The NETRA command center runs the final detection and the operator validation.

| Node                    | Role                                                                                              |
| ----------------------- | ------------------------------------------------------------------------------------------------- |
| `anomaly_detector_node` | answers the path follower's inspection hook at every stop point; optionally inspects all the time |
| `person_detector_node`  | YOLO11n people on the camera: counted, boxed for the handheld, placed on the map floor            |

## anomaly_detector_node

| Interface                                          | Type                                         |                                                                                                                                                                                        |
| -------------------------------------------------- | -------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `image_topic` (`/camera/camera/color/image_raw`)   | `sensor_msgs/Image`                          | camera (rgb8, bgr8, rgba8, bgra8, mono8)                                                                                                                                               |
| `~/check_now` (`/anomaly_detector_node/check_now`) | `std_srvs/Trigger`                           | inspect the latest frame now: the path follower's `hooks.inspection` calls it at every stop point; switched off it answers `inspection off` at once (success), so a patrol never waits |
| `anomaly_detector/result`                          | `std_msgs/String` (JSON)                     | `{category, description, model, latency_ms, is_anomaly, source, captured_at, evidence_stamp}`, only for `alarm_categories`; `pashupati_fleet` uploads it with the evidence and pose    |
| `anomaly_detector/evidence`                        | `sensor_msgs/CompressedImage`                | the analysed frame, published just before the result; its stamp is `evidence_stamp`                                                                                                    |
| `anomaly_detector/status`                          | `std_msgs/String` (JSON, latched, every 2 s) | `{enabled, model, state (loading, running, off, error), error, interval_sec, models, categories, last, inspections, reports, queue, queue_max}`, shown on the handheld                 |

A stop point whose inspection node is not running is not held up either: the follower calls only
the hooks that are available (`INSPECTION=false` on a light robot).

The model picks one of `categories` (default `trash`, `spill`, `stain`) or `none`. Only
`alarm_categories` (default `trash`, `spill`) are reported; `stain` is too easily confused with
reflections on shiny floors. With `interval_sec: 0` the node inspects back to back and reports a
type at most once per `cooldown_sec`; a stop point always reports.

**Live control.** `enabled`, `model`, `interval_sec`, `categories`, `alarm_categories` and
`category_descriptions` change while the node runs (handheld, NETRA, `ros2 param set`); every
other parameter is read-only. A new model is loaded and the one before it unloaded (the Jetson
shares its memory with the GPU); off, or a stopped node, unloads it too. The live choice is kept
in `state_file` (`~/.pashupati/inspection.json`) and wins over the parameter files at the next
start; delete the file to go back to them.

```bash
ros2 param set /anomaly_detector_node model moondream:latest
ros2 param set /anomaly_detector_node categories "[trash, spill, bag]"
ros2 param set /anomaly_detector_node category_descriptions "['bag: a bag or suitcase left alone']"
ros2 service call /anomaly_detector_node/check_now std_srvs/srv/Trigger
```

**Interval queue.** With `interval_sec` above 0 a frame is taken every `interval_sec` seconds,
also while the model is busy, and waits in a first-in-first-out queue (`queue_dir`, kept across
restarts, at most `queue_max`). A frame that looks like the last queued one (`queue_min_change`)
is skipped. Findings from the queue carry the time they were taken.

**Memory.** Ollama's runner keeps earlier prompts in RAM, up to 8 GiB per model, useless when
every prompt has a new image: `scripts/setup.sh install ollama` turns that cache off.

## person_detector_node

YOLO11n exported to ONNX at 416 px (`scripts/setup.sh install people_model`), run by ONNX
Runtime (CUDA when available) or OpenCV DNN, no PyTorch; `.pt` / `.engine` go through
Ultralytics. `rate_hz: 0` detects 5 times a second on the GPU, 2 on the CPU. People are placed on
the floor (`robot.base_height` below `base_link`) in the map frame while the robot is localized.
`enabled`, `rate_hz` and `confidence` are live; on / off is kept in `~/.pashupati/people.json`.

| Topic                      | Type                              |                                                                                          |
| -------------------------- | --------------------------------- | ---------------------------------------------------------------------------------------- |
| `perception/people`        | `geometry_msgs/PoseArray`         | people on the floor, map frame; `orientation.w` is the score                             |
| `perception/people_status` | `std_msgs/String` (JSON, latched) | `{enabled, state, error, backend, model, count, boxes, placed, latency_ms, at, rate_hz}` |

## Layout

| Module                                                                   | What                                                                  |
| ------------------------------------------------------------------------ | --------------------------------------------------------------------- |
| `anomaly_detector_node.py`                                               | parameters, topics, the stop-point service, the worker thread         |
| `person_detector_node.py`                                                | parameters, topics, placing people with TF                            |
| `core/config.py`                                                         | `AnomalyConfig`, `PeopleConfig` (typed, validated, live or read-only) |
| `core/vlm.py`                                                            | prompt, JSON schema, answer parsing, model options                    |
| `core/ollama.py`                                                         | the Ollama HTTP API: load, unload, ask, models                        |
| `core/engine.py`                                                         | the loaded model, what an answer means (report, cooldown), the status |
| `core/loop.py`                                                           | the worker: back to back, interval queue, off frees the model         |
| `core/frame_queue.py`                                                    | the interval queue on disk                                            |
| `core/choice.py`                                                         | live choices kept across restarts                                     |
| `core/images.py`                                                         | ROS image bytes to BGR, JPEG                                          |
| `core/person_detector.py`, `core/person_projection.py`, `core/people.py` | YOLO pre / post-processing, the floor projection, the people status   |

## Launch and test

```bash
ros2 launch pashupati_inspection inspection.launch.py                  # both nodes
ros2 launch pashupati_inspection inspection.launch.py people:=false camera_image:=/cam/image_raw
colcon test --packages-select pashupati_inspection && colcon test-result --verbose
```

Arguments: `params_file`, `robot_params`, `use_sim_time`, `log_level` (the Dharma contract),
`anomaly`, `people`, `camera_image`, `camera_info`, `ollama_url`. Defaults:
[config/inspection.yaml](config/inspection.yaml). Requires Ollama >= 0.12.7 for `qwen3-vl`.

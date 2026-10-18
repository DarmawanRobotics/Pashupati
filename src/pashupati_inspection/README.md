# pashupati_inspection

`anomaly_detector_node` — early anomaly detection on the robot with a local vision-language model
(Ollama, no cloud). The NETRA command center runs the final detection and the operator validation.

| Interface | Type | |
|---|---|---|
| `image_topic` (`/camera/camera/color/image_raw`) | `sensor_msgs/Image` | camera input |
| `~/check_now` (`/anomaly_detector_node/check_now`) | `std_srvs/Trigger` | inspect the latest frame now; called by `path_follower_node` at every stop point |
| `anomaly_detector/result` | `std_msgs/String` (JSON) | `{category, description, model, latency_ms, is_anomaly, source, evidence_stamp}`, published only for `alarm_categories`; `pashupati_fleet` uploads them with the evidence frame and pose |
| `anomaly_detector/evidence` | `sensor_msgs/CompressedImage` | the camera frame the result was detected in (full size), published just before the result; its stamp equals `evidence_stamp` |
| `anomaly_detector/status` | `std_msgs/String` (JSON, latched, every 2 s and on change) | `{enabled, model, state (loading, running, off, error), error, interval_sec, models [{name, installed, active}], last {category, reported, latency_ms, description, source, at}, inspections, reports}`, shown on the handheld |

The model picks one of `categories` (default `trash`, `spill`, `stain`) or `none`. Only
`alarm_categories` (default `trash`, `spill`) are reported; `stain` is too easily confused with
reflections on shiny floors. With `interval_sec: 0` the node inspects back to back and reports a
category at most once per `cooldown_sec`; `check_now` (source `stop_point`) always reports.

### Own finding types

`categories`, `alarm_categories` and `category_descriptions` change live too. On the handheld
(Inspection page, *Finding types*) a type is added with a name and a description of what the
model should look for, switched between reported and only shown, edited or removed. The model
reads every type with its description in the prompt, so a short concrete English description
works best, e.g. `bag: a bag, backpack or suitcase standing alone on the floor`. Up to 12 types
of up to 200 characters; `trash`, `spill`, `stain`, `floor_damage` and `fallen_person` have
built-in descriptions. Changed live they are kept in `state_file` and win over the parameter
files.

```bash
ros2 param set /anomaly_detector_node categories "[trash, spill, bag]"
ros2 param set /anomaly_detector_node category_descriptions "['bag: a bag or suitcase left alone']"
ros2 param set /anomaly_detector_node alarm_categories "[trash, spill, bag]"
```

## Live control

`enabled`, `model`, `interval_sec` and the finding types change while the node runs: from the handheld (Inspection
page) or with `ros2 param set /anomaly_detector_node model moondream:latest`. A new model is
loaded into Ollama and the one before it unloaded (the Jetson shares its memory with the GPU).
Off, or a stopped node, unloads the model too (`keep_alive` holds it only while inspecting), so
turning it on again takes a few seconds of loading. The live choice is kept in `state_file`
(`~/.pashupati/inspection.json`) and wins over the parameter files at the next start; delete that
file to go back to them.

**Interval queue.** With `interval_sec` above 0 a camera frame is taken every `interval_sec`
seconds, also while the model is still busy, and waits in a queue the model works through first
in first out whenever it is free (between frames, while the robot stands at a stop). Frames are
kept in `queue_dir` across restarts; at `queue_max` the oldest one makes room. A frame that looks
the same as the last queued one (`queue_min_change`, robot standing still) is not queued again.
Findings from the queue go to NETRA with the time and the robot pose of when the frame was taken.
The handheld shows the queue (`queue` in the status).

**Memory.** Ollama's runner (llama-server) keeps earlier prompts in RAM, up to 8 GiB per loaded
model, which an inspection never reuses: every prompt has a new image. `script/setup.sh install
ollama` turns that cache off for the Ollama service (`/etc/systemd/system/ollama.service.d/
pashupati.conf`); without it `llama-server` grows to about 9 GB. `ollama ps` shows the model,
`ps -o rss= -C llama-server` the runner's RAM.

The handheld offers the `models` parameter (known to work on the robot: `qwen3-vl:2b-instruct`,
`moondream:latest`, `ahmadwaqar/smolvlm2-2.2b-instruct:latest`) plus every other vision model
Ollama has; models not pulled yet are marked so (`script/setup.sh install ollama` pulls these
three).

Debug parameters: `model`, `use_schema` (false = free-text `FLOOR:`/`TYPE:` answer, for models
whose runner crashes on grammar), `describe` (one sentence about the floor) and `debug` (log every
inspection and the raw answer).

## Run

```bash
ros2 launch pashupati_inspection inspection.launch.py
ros2 run pashupati_inspection anomaly_detector_node --ros-args \
  -p model:=ahmadwaqar/smolvlm2-2.2b-instruct:latest -p describe:=true -p debug:=true
ros2 service call /anomaly_detector_node/check_now std_srvs/srv/Trigger
ros2 topic echo /anomaly_detector/result
```

Requires Ollama >= 0.12.7 for `qwen3-vl`.

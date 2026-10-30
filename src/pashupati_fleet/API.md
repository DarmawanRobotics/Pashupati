# NETRA robot API (v1)

Pashupati 3 (on Dharma 0.2) keeps this protocol. Changes against 2.x, all additive except one:

- **Changed:** the `alert` states `TF_UNAVAILABLE` and `AVOIDANCE_STALE` are no longer sent. On
  Dharma 0.2 the follower holds the robot for a stale pose or a stale avoidance without leaving
  its state; the reason is in `nav.message` (`TF_STALE: ...`, `AVOIDANCE_STALE: ...`). New alert
  state `FAILED` (the follower gave up).
- New: `nav.state` `FAILED`; `health.level` (`ok`, `warn`, `error`) and `health.speed_scale`;
  the commands `dock` and `undock`; `site` in the route upload meta.
- `mission` is `lap <n>` while a looping patrol from NETRA runs (was the follower's behaviour
  text), `localization` is `none` or the source of the last correction (`tag:base_map`, ...).

Contract between a robot (`fleet_uplink_node`) and the NETRA command center. Every request carries
`Authorization: Bearer <robot token>` and `X-Robot-Id: <robot_id>`; the server maps the token to a
robot and rejects mismatches. All times are Unix seconds (float), positions are metres in the
robot's `map` frame, yaw in radians unless the field says `_deg`.

## WebSocket `GET /api/v1/robot/ws`

One connection per robot, JSON text frames. The robot reconnects with backoff; events produced
while offline are replayed after `hello` (telemetry is not).

### Robot → server

| `type`             | Fields                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| ------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `hello`            | `robot_id`, `version`                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| `telemetry` (1 Hz) | `ts`, `pose` {x, y, yaw} \| null, `mode` (`auto`/`remote`), `estop` (bool), `field` (protective field state), `nav` {state (`NAV_INACTIVE`, `JOINING`, `FOLLOWING`, `APPROACHING`, `DWELLING`, `PAUSED`, `BLOCKED`, `OFF_PATH`, `GOAL_REACHED`, `FAILED`), message}, `mission`, `localization`, `battery` {pct, v, a, temp} \| null, `driver` {level, msg} \| null, `people` (count in view), `inspection` / `people_detection` {enabled, state (`loading`, `running`, `off`, `error`), error, model?, interval_sec?, count?, backend?} \| null (the detectors' status; null when not running), `health` {ok, level, issues, speed_scale, topics ({topic: Hz}), cpu_temp_c, memory_percent, memory_used_gb, memory_total_gb, cpu_percent, gpu_percent, disk_free_gb, load} \| null, `patrol` {patrol_id, route_id} \| null |
| `patrol_event`     | `event` (`started`, `lap`, `finished`), `lap?`, `reason?`, `patrol_id`, `route_id`, `ts`                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| `stop_event`       | `index`, `event` (`arrived`, `inspected`, `departed`, `skipped`), `message`, `x`, `y`, `yaw_deg`, `patrol_id`, `route_id`, `ts`                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| `alert`            | `state` (`BLOCKED`, `OFF_PATH`, `FAILED`, `ESTOP` with the e-stop reasons in `message`), `message`, `pose`, `patrol_id`, `route_id`, `ts`                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| `crowd`            | `ts_start`, `ts_end`, `cell_size` (m), `cells` [[ix, iy, peak_people], ...] (cell = floor(x / cell_size)), `peak_people`, `frames`, `patrol_id`, `route_id`, `ts`                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| `ack`              | `id`, `ok`, `msg`                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |

### Server → robot

`{"type": "command", "id": <unique>, "name": <name>, "args": {...}}`, answered by one `ack`.

| `name`            | `args`                                                                                                             | Robot behaviour                                                                                                                                                      |
| ----------------- | ------------------------------------------------------------------------------------------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `start_patrol`    | `route_id`, `patrol_id`                                                                                            | download the route file into the active site (`<site>/netra/<route_id>.csv`; refused without an active site), load it, start navigation, emit `patrol_event started` |
| `stop_patrol`     |                                                                                                                    | stop navigation                                                                                                                                                      |
| `pause`, `resume` |                                                                                                                    | hold / continue navigation                                                                                                                                           |
| `localize`        |                                                                                                                    | localize from a visible AprilTag                                                                                                                                     |
| `estop`           |                                                                                                                    | latch the software e-stop (robot holds still, navigation pauses)                                                                                                     |
| `estop_release`   |                                                                                                                    | release it; navigation stays paused until `resume`                                                                                                                   |
| `set_inspection`  | any of `enabled` (bool), `model` (Ollama model), `interval` (s: 0 continuous, > 0 every N s, < 0 stop points only) | VLM inspection live (kept across robot restarts)                                                                                                                     |
| `set_people`      | `enabled` (bool)                                                                                                   | person detection on or off (kept across robot restarts)                                                                                                              |
| `dock`, `undock`  |                                                                                                                    | dock onto the charger in view, leave it (`docking/dock`, `docking/undock`)                                                                                           |

## HTTP uploads (multipart/form-data)

Field `meta` holds a JSON object with a client-generated `uid`; uploads are retried until the
server answers 2xx, so the server must treat a known `uid` as success (2xx or 409).

### `POST /api/v1/robot/anomalies`

Files: `image.jpg`. `meta`:

| Field                         |                                                                                             |
| ----------------------------- | ------------------------------------------------------------------------------------------- |
| `uid`                         | idempotency key                                                                             |
| `category`                    | robot's early classification, e.g. `trash`, `spill`, `floor_damage`, `fallen_person`        |
| `confidence`                  | 0..1                                                                                        |
| `description`                 | detector text                                                                               |
| `pose`                        | {x, y, yaw} \| null                                                                         |
| `evidence`                    | `analysed_frame` (the image is the frame the robot classified) or `latest_frame` (fallback) |
| `stop_index`                  | stop point index when taken at a stop, else null                                            |
| `patrol_id`, `route_id`, `ts` | context                                                                                     |
| `detector`                    | `robot`                                                                                     |

### `POST /api/v1/robot/routes`

Files: `route.csv` (`x,y,yaw_deg,dwell_sec` rows). `meta`: `uid`, `name`, `recorded_at`,
`source_path`, `site`, `length_m`, `points`, `stops`, `closed`.

### `GET /api/v1/robot/routes/{route_id}/file`

Returns the route CSV for `start_patrol`.

## Anomaly detector input

`fleet_uplink_node` listens to `anomaly_topic` (`std_msgs/String`, JSON). Accepted shapes:
`{"is_anomaly"|"anomaly"|"detected": bool, "category"|"label"|"class"|"type": str,
"confidence"|"score": float, "description"|"reason": str}`. Results with no anomaly, category
`none`/`normal`, or confidence below `anomaly_min_confidence` are ignored; the latest frame from
`anomaly_image_topic` (≤ 3 s old) is attached.

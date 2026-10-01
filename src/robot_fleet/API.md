# NETRA robot API (v1)

Contract between a robot (`fleet_uplink_node`) and the NETRA command center. Every request carries
`Authorization: Bearer <robot token>` and `X-Robot-Id: <robot_id>`; the server maps the token to a
robot and rejects mismatches. All times are Unix seconds (float), positions are metres in the
robot's `map` frame, yaw in radians unless the field says `_deg`.

## WebSocket `GET /api/v1/robot/ws`

One connection per robot, JSON text frames. The robot reconnects with backoff; events produced
while offline are replayed after `hello` (telemetry is not).

### Robot → server

| `type` | Fields |
|---|---|
| `hello` | `robot_id`, `version` |
| `telemetry` (1 Hz) | `ts`, `pose` {x, y, yaw} \| null, `mode` (`auto`/`remote`), `nav` {state, message}, `mission`, `localization`, `battery` {pct, v, a, temp} \| null, `driver` {level, msg} \| null, `people` (count in view), `health` {ok, issues, topics, cpu_temp_c, memory_percent, disk_free_gb, load} \| null, `patrol` {patrol_id, route_id} \| null |
| `patrol_event` | `event` (`started`, `lap`, `finished`), `lap?`, `reason?`, `patrol_id`, `route_id`, `ts` |
| `stop_event` | `index`, `event` (`arrived`, `inspected`, `departed`, `skipped`), `message`, `x`, `y`, `yaw_deg`, `patrol_id`, `route_id`, `ts` |
| `alert` | `state` (`BLOCKED`, `OFF_PATH`, `TF_UNAVAILABLE`, `AVOIDANCE_STALE`), `message`, `pose`, `patrol_id`, `route_id`, `ts` |
| `crowd` | `ts_start`, `ts_end`, `cell_size` (m), `cells` [[ix, iy, peak_people], ...] (cell = floor(x / cell_size)), `peak_people`, `frames`, `patrol_id`, `route_id`, `ts` |
| `ack` | `id`, `ok`, `msg` |

### Server → robot

`{"type": "command", "id": <unique>, "name": <name>, "args": {...}}`, answered by one `ack`.

| `name` | `args` | Robot behaviour |
|---|---|---|
| `start_patrol` | `route_id`, `patrol_id` | download the route file, load it, start navigation, emit `patrol_event started` |
| `stop_patrol` | | stop navigation |
| `pause`, `resume` | | hold / continue navigation |
| `localize` | | localize from a visible AprilTag |

## HTTP uploads (multipart/form-data)

Field `meta` holds a JSON object with a client-generated `uid`; uploads are retried until the
server answers 2xx, so the server must treat a known `uid` as success (2xx or 409).

### `POST /api/v1/robot/anomalies`

Files: `image.jpg`. `meta`:

| Field | |
|---|---|
| `uid` | idempotency key |
| `category` | robot's early classification, e.g. `trash`, `spill`, `floor_damage`, `fallen_person` |
| `confidence` | 0..1 |
| `description` | detector text |
| `pose` | {x, y, yaw} \| null |
| `stop_index` | stop point index when taken at a stop, else null |
| `patrol_id`, `route_id`, `ts` | context |
| `detector` | `robot` |

### `POST /api/v1/robot/routes`

Files: `route.csv` (`x,y,yaw_deg,dwell_sec` rows). `meta`: `uid`, `name`, `recorded_at`,
`source_path`, `length_m`, `points`, `stops`, `closed`.

### `GET /api/v1/robot/routes/{route_id}/file`

Returns the route CSV for `start_patrol`.

## Anomaly detector input

`fleet_uplink_node` listens to `anomaly_topic` (`std_msgs/String`, JSON). Accepted shapes:
`{"is_anomaly"|"anomaly"|"detected": bool, "category"|"label"|"class"|"type": str,
"confidence"|"score": float, "description"|"reason": str}`. Results with no anomaly, category
`none`/`normal`, or confidence below `anomaly_min_confidence` are ignored; the latest frame from
`anomaly_image_topic` (≤ 3 s old) is attached.

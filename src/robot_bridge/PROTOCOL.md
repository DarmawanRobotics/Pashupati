# Teleop UDP protocol (v1)

One UDP port on the robot (default `9870`). The handheld sends JSON datagrams; the robot answers
with JSON datagrams (control) and chunked binary datagrams (streams). No ROS on the handheld.

## Session

1. Client sends `hello` (and again every 1 s as a keep-alive when idle).
2. Robot replies `welcome` and the loaded route, then streams `state` at `telemetry_hz`.
3. Any datagram refreshes the session; after `client_timeout_sec` of silence the robot drops the
   client and zeroes the remote velocity. A new `hello` from another address takes over.

If the robot is configured with a `token`, every client message must include `"k": "<token>"`.

## Client → robot

| `t` | Fields | Meaning |
|---|---|---|
| `hello` | `name`, `streams: {camera: bool, lidar: bool}` | register / keep alive |
| `joy` | `vx`, `vy` (m/s), `wz` (rad/s) | teleop velocity, send at 20–30 Hz; only used in `remote` mode |
| `cmd` | `id` (int, unique per command), `name`, `args` (object) | run a command, see below |
| `streams` | `camera`, `lidar` (bool) | change subscriptions |
| `ping` | `ts` | latency probe, echoed as `pong` |
| `bye` | | leave |

Commands are idempotent per `id`: resend the same `cmd` until its `ack` arrives (e.g. every 200 ms);
the robot runs it once and resends the cached `ack`.

| `name` | `args` | Effect |
|---|---|---|
| `set_mode` | `mode`: `auto` \| `remote` | choose who drives; `remote` pauses navigation |
| `nav_start`, `nav_stop` | | start / stop navigation |
| `nav_pause`, `nav_resume` | | hold / continue without losing progress |
| `record_start`, `record_stop` | | route recording |
| `mark_stop` | `dwell` (s) | mark an inspection stop at the current pose |
| `list_routes` | | `ack.data` = route CSV paths, newest first |
| `load_route` | `file` | load a route CSV |
| `get_route` | | resend the loaded route stream |
| `localize` | | localize from a visible AprilTag |
| `record_tags` | | record AprilTag poses |
| `stand_up`, `sit_down`, `emergency_stop`, `move_mode`, `balance_stand_mode`, `lock_mode`, `slow_speed`, `normal_speed`, `fast_speed` | | robot driver commands (`robot_commands` parameter) |

## Robot → client (JSON)

| `t` | Fields |
|---|---|
| `welcome` | `commands` (list), `limits` ([vx, vy, wz] maxima) |
| `ack` | `id`, `ok` (bool, `null` while pending), `msg`, optional `data` |
| `state` | `mode`, `nav`, `nav_msg`, `mission`, `localization`, `recording`, `pose` ([x, y, yaw] in map or `null`), `battery` ({pct, v, a, temp} or `null`), `driver` ({level, msg} or `null`), `cmd` ([vx, vy, wz] sent to the robot) |
| `pong` | `ts` |
| `bye` | robot shutting down |

## Binary streams

Every datagram starts with a 13-byte little-endian header:

| Offset | Type | Field |
|---|---|---|
| 0 | 4 bytes | magic `PSH1` |
| 4 | u8 | stream: 1 camera, 2 lidar, 3 route |
| 5 | u32 | frame number (increasing per stream) |
| 9 | u16 | chunk index |
| 11 | u16 | chunk count |

Payloads are split into chunks of at most 1400 bytes (no IP fragmentation). Reassemble per stream;
when a chunk of a newer frame arrives, drop the incomplete older frame.

| Stream | Payload |
|---|---|
| 1 camera | one JPEG image |
| 2 lidar | N × (int16 x, int16 y, int16 z) in centimetres, robot frame (`base_link`), little-endian |
| 3 route | JSON `{"t": "route", "points": [[x, y, dwell_sec], ...]}` in the map frame |

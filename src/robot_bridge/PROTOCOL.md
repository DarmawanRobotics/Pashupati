# Teleop UDP protocol (v1)

One UDP port on the robot (default `9870`). The handheld sends JSON datagrams; the robot answers
with JSON datagrams (control) and chunked binary datagrams (streams). No ROS on the handheld.

## Session

1. Client sends `hello` (and again every 1 s as a keep-alive when idle).
2. Robot replies `welcome` and the loaded route, then streams `state` at `telemetry_hz`.
3. Any datagram refreshes the session; after `client_timeout_sec` of silence the robot drops the
   client and zeroes the remote velocity.
4. One driver at a time: a `hello` from another device (IP address) is answered with `busy` while
   the current client is alive. The same device may reconnect from a new port (app restart).

If the robot is configured with a `token`, every client message must include `"k": "<token>"`
(compared in constant time). Without a token anyone on the network can drive the robot; the robot
logs a warning at start.

Malformed datagrams are dropped. Joystick values that are not finite numbers (NaN, inf, strings,
booleans) count as 0, so a broken client stops the robot instead of driving it at full speed.

## Discovery

Any device may broadcast `{"t": "discover"}` to UDP port 9870 (no token needed). Every robot answers
the sender with `{"t": "beacon", "name", "secured" (token required), "in_use_by" (name of the
connected handheld or ""), "mode", "battery"}`; the handheld lists robots without typing an IP.

## Client → robot

| `t` | Fields | Meaning |
|---|---|---|
| `hello` | `name`, `streams: {camera: bool, lidar: bool, camera_fps?: number}` | register / keep alive |
| `joy` | `vx`, `vy` (m/s), `wz` (rad/s) | teleop velocity, send at 20–30 Hz; only used in `remote` mode |
| `cmd` | `id` (int, unique per command), `name`, `args` (object) | run a command, see below |
| `streams` | `camera`, `lidar` (bool), `camera_fps` (number, capped by the `camera_fps` parameter) | change subscriptions and camera rate |
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
| `mark_stop` | `dwell` (s, 0–3600) | mark an inspection stop at the current pose |
| `list_routes` | | `ack.data` = route CSV paths, newest first |
| `load_route` | `file` | load a route CSV; only files under `routes_dir` |
| `get_route` | | resend the loaded route stream |
| `localize` | | localize from a visible AprilTag |
| `record_tags` | | record AprilTag poses |
| `stand_up`, `sit_down`, `emergency_stop`, `move_mode`, `balance_stand_mode`, `lock_mode`, `slow_speed`, `normal_speed`, `fast_speed` | | robot driver commands (`robot_commands` parameter) |

## Robot → client (JSON)

| `t` | Fields |
|---|---|
| `welcome` | `commands` (list), `limits` ([vx, vy, wz] maxima) |
| `ack` | `id`, `ok` (bool, `null` while pending), `msg`, optional `data` |
| `state` | `seq` (increments per state, for packet-loss estimates), `mode`, `nav`, `nav_msg`, `mission`, `localization`, `recording`, `pose` ([x, y, yaw] in map or `null`), `battery` ({pct, v, a, temp} or `null`), `driver` ({level, msg} or `null`), `cmd` ([vx, vy, wz] sent to the robot), `health` ({ok, issues[]} or `null`) |
| `pong` | `ts` |
| `busy` | `msg`: another device is driving, retry later |
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

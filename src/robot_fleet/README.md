# robot_fleet

Robot side of the NETRA command center. `fleet_uplink_node` keeps one WebSocket to the server and
an on-disk outbox for uploads, so a robot that loses Wi-Fi in the middle of a patrol loses nothing.

| Data | Source | Sent as |
|---|---|---|
| Telemetry (1 Hz) | navigation, localization, mode, battery, driver diagnostics, TF pose | WebSocket `telemetry` |
| Patrol, stop-point and alert events | `navigation/status`, `navigation/mission_status`, `navigation/stop_point_event` | WebSocket, replayed after reconnect |
| Crowd heatmap | `perception/people` (`person_detector_node`) → peak people per 1 m cell per minute | WebSocket `crowd` |
| Anomalies | anomaly detector JSON + latest camera frame + pose | HTTP upload, spooled |
| Recorded routes | `mapping/route_saved` | HTTP upload, spooled |

Commands from the server (`start_patrol`, `stop_patrol`, `pause`, `resume`, `localize`) run on the
ROS thread and are acknowledged. The wire contract is [API.md](API.md).

## Run

```bash
ros2 launch robot_fleet fleet.launch.py robot_ns:=l1w \
  server_url:=https://netra.example.com token:=<robot token>
```

Without `server_url` the node only spools uploads (`~/.pashupati/spool`). Parameters:
[config/fleet_params.yaml](config/fleet_params.yaml).

## Test

```bash
colcon test --packages-select robot_fleet && colcon test-result --verbose
```

# pashupati_fleet

Robot side of the NETRA command center. `fleet_uplink_node` keeps one WebSocket to the server and
an on-disk spool for uploads, so a robot that loses Wi-Fi in the middle of a patrol loses nothing.
The wire contract is [API.md](API.md) (v1).

| Data                   | Source (Dharma 0.2)                                                                                                  | Sent as                             |
| ---------------------- | -------------------------------------------------------------------------------------------------------------------- | ----------------------------------- |
| Telemetry (1 Hz)       | `control/status`, `navigation/status`, `localization/status`, `health/status` + `/diagnostics`, battery, driver, TF  | WebSocket `telemetry`               |
| Patrol and stop events | `navigation/status` (laps, finish), `navigation/stop_point`                                                          | WebSocket, replayed after reconnect |
| Alerts                 | follower `BLOCKED`, `OFF_PATH`, `FAILED`; the control mux's e-stop with its reasons                                  | WebSocket `alert`                   |
| Crowd heatmap          | `perception/people` → peak people per 1 m cell per minute                                                            | WebSocket `crowd`                   |
| Anomalies              | `anomaly_detector/result` + the frame it was found in (`anomaly_detector/evidence`, latest frame as fallback) + pose | HTTP upload, spooled                |
| Taught routes          | `route/saved` (dharma_mapping)                                                                                       | HTTP upload, spooled                |

NETRA's commands become the stack's services: `start_patrol` (route downloaded into the active
site, `navigation/load_path`, `navigation/start`), `stop_patrol`, `pause`, `resume`
(`navigation/*`), `localize`, `estop` (`control/estop_request` at once, then
`control/estop/engage`), `estop_release`, `dock`, `undock`, `set_inspection`, `set_people`. Each
is acknowledged.

## fake_netra

A stand-in NETRA (standard library only) for the simulator and the tests: it accepts the
robot's WebSocket, records what it sends, answers uploads and serves route files.

```bash
ros2 run pashupati_fleet fake_netra --port 8765 --log /tmp/netra.jsonl
ros2 launch pashupati sim.launch.py cc_url:=http://127.0.0.1:8765
```

## Layout

| Module                                                                                                     |                                                                                                       |
| ---------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------- |
| `fleet_uplink_node.py`                                                                                     | subscriptions, commands, spooling                                                                     |
| `core/patrol.py`                                                                                           | the patrol: start, laps, alerts, finish, stop events                                                  |
| `core/anomalies.py`                                                                                        | findings waiting for their frame                                                                      |
| `core/crowd.py`, `core/poses.py`, `core/spool.py`, `core/routes.py`, `core/switches.py`, `core/anomaly.py` | heatmap, pose history, the outbox on disk, route ids and summaries, detector switches, result parsing |
| `utils/connection.py`                                                                                      | WebSocket (reconnect, outbox) and the uploader threads                                                |
| `utils/telemetry.py`                                                                                       | the status subscriptions                                                                              |
| `tools/fake_netra.py`                                                                                      | the stand-in server                                                                                   |

## Launch and test

```bash
ros2 launch pashupati_fleet fleet.launch.py robot_ns:=robot server_url:=https://netra.example.com token:=<token>
colcon test --packages-select pashupati_fleet && colcon test-result --verbose
```

Without `server_url` the node only spools (`~/.pashupati/spool`). Parameters:
[config/fleet.yaml](config/fleet.yaml), `ros2 param describe /fleet_uplink_node <name>`.

# robot_health

`health_monitor_node` watches what a patrol depends on and says when something is off.

- **Sensor rates**: every `topic:min_hz` in `topics` is subscribed raw (no deserialisation, cheap
  even for point clouds) as soon as it appears; below 80 % of the minimum is an issue.
- **Computer**: hottest thermal zone, memory use, free disk at `disk_path`, load average.

| Topic | Type | |
|---|---|---|
| `pashupati/diagnostics` | `diagnostic_msgs/DiagnosticArray` | one status per check (rqt_robot_monitor) |
| `health/status` | `std_msgs/String` (JSON, latched) | `{ok, issues[], topics{topic: hz}, cpu_temp_c, memory_percent, disk_free_gb, load}` — shown on the teleop and sent to NETRA with the telemetry |

```bash
ros2 launch robot_health health.launch.py
ros2 topic echo /health/status
```

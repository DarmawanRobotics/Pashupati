# robot_mapping

FAST-LIO2 bringup and teach-and-repeat route recording.

- `mapping.launch.py` starts FAST-LIO (`save_map:=false` while patrolling, otherwise every scan is
  kept in RAM), the localization frame bridge (`tags_config_file:=`) and `path_recorder_node`.
- `path_recorder_node` samples `map → base_link` while recording and writes
  `map/<date>/<time>_path.csv` (`x,y,yaw_deg,dwell_sec`).

| Service / topic | Type | |
|---|---|---|
| `mapping/path_record` | `std_srvs/SetBool` | start / stop recording |
| `mapping/mark_stop_point` | `robot_interfaces/MarkStopPoint` | stop point at the current pose |
| `mapping/route_saved` | `std_msgs/String` | path of the CSV just written |

Parameters: [config/mapping_param.yaml](config/mapping_param.yaml).

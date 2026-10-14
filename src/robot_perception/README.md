# robot_perception

| Node | Output |
|---|---|
| `lidar_sector_node` | `perception/sector_scan` — nearest obstacle per angular sector in `base_link` (Livox CustomMsg or any PointCloud2), floor and robot-body filtered, front `fov_deg` for avoidance; `perception/safety_scan` — the same over 360° (`safety_num_sectors`) for the protective field, every sector reported blocked (0 m) when a cloud has fewer than `min_raw_points` points (LiDAR covered or failed) |
| `apriltag_node` (apriltag_ros) | tag TFs used by localization |
| `person_detector_node` | `perception/people` — people on the floor in the `map` frame (YOLO + floor-plane projection, no depth needed); feeds the crowd heatmap |

`input_type: livox` (default) reads the Livox CustomMsg straight from its serialized bytes with
numpy (`livox_cdr.py`, ~1 ms per scan); `pointcloud2` takes any PointCloud2, e.g. for another
LiDAR. FAST-LIO's `/cloud_registered_body` misses everything within its `blind` radius, so it is
not a good input for obstacles next to the robot.

The person detector needs `pip install ultralytics` and a model (`.pt`, `.onnx` or a TensorRT
`.engine` exported on the Jetson) at `model_path`; set `floor_z` to minus the height of `base_link`.

```bash
ros2 launch robot_perception perception.launch.py people:=true draw_tags:=false
```

Parameters: [config/perception_params.yaml](config/perception_params.yaml).

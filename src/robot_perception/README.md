# robot_perception

| Node | Output |
|---|---|
| `lidar_sector_node` | `perception/sector_scan` — nearest obstacle per angular sector in `base_link` (Livox CustomMsg or any PointCloud2), floor and robot-body filtered |
| `apriltag_node` (apriltag_ros) | tag TFs used by localization |
| `person_detector_node` | `perception/people` — people on the floor in the `map` frame (YOLO + floor-plane projection, no depth needed); feeds the crowd heatmap |

The person detector needs `pip install ultralytics` and a model (`.pt`, `.onnx` or a TensorRT
`.engine` exported on the Jetson) at `model_path`; set `floor_z` to minus the height of `base_link`.

```bash
ros2 launch robot_perception perception.launch.py people:=true draw_tags:=false
```

Parameters: [config/perception_params.yaml](config/perception_params.yaml).

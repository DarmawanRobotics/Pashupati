# molly_perception

Lidar sector perception buat Molly. Konsumsi **livox_ros_driver2/msg/CustomMsg**
langsung (native Livox format) -- **bukan** `sensor_msgs/PointCloud2`. Keluarin
`molly_navigation_msgs/SectorScan` (default 32 sektor, front 180°: -90..+90 deg)
+ `MarkerArray` buat RViz.

Murni deteksi/representasi -- gak ada logic avoidance/danger di sini (itu
punya `molly_navigation`'s `obstacle_avoidance_node`).

## Kenapa CustomMsg, bukan PointCloud2

Livox driver (`livox_ros_driver2`) bisa publish 2 format di topic yang sama
(`/livox/lidar`), diatur lewat parameter `xfer_format` di launch file driver:

| `xfer_format` | Format | Tipe msg |
|---|---|---|
| `0` (default) | Livox pointcloud2 (PointXYZRTLT) | `sensor_msgs/PointCloud2` |
| `1` | **Livox customized pointcloud** | `livox_ros_driver2/msg/CustomMsg` |
| `2` | Standard PCL pointcloud2 | `sensor_msgs/PointCloud2` |

Node ini pakai mode `1`. `CustomMsg` udah berupa array `CustomPoint[]`
(field: `x, y, z, reflectivity, tag, line, offset_time`) -- gak perlu decode
byte-buffer PointCloud2 (`sensor_msgs_py.point_cloud2`), lebih ringan buat
Jetson.

**Trade-off:** node ini jadi Livox-specific. Kalau nanti ganti sensor selain
Livox, balik ke versi PointCloud2 (lihat riwayat sebelumnya di repo/chat).

## Prasyarat: livox_ros_driver2

`livox_ros_driver2` **gak ada di apt**, harus di-clone & build manual di
workspace yang sama:

```bash
cd ~/molly_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
# ikuti instruksi build di README repo itu (butuh Livox-SDK2 terinstall dulu)
cd ~/molly_ws
colcon build --packages-select livox_ros_driver2
```

Launch driver-nya dengan `xfer_format:=1` (pakai `msg_MID360.launch` sebagai
referensi dari repo Livox, bukan `rviz_MID360.launch` yang defaultnya
PointCloud2).

## Packages
| Package | Type | Isi |
|---|---|---|
| `molly_navigation_msgs` | ament_cmake | `SectorScan`, `AvoidanceCommand` |
| `livox_ros_driver2` | ament_cmake (external, clone manual) | driver + `CustomMsg`/`CustomPoint` |
| `molly_perception` | ament_python | `lidar_sector_perception_node` (package ini) |

## Node
| Node | Subscribe | Publish |
|---|---|---|
| `lidar_sector_perception_node` | `livox_ros_driver2/msg/CustomMsg` (`/livox/lidar`) | `molly_navigation_msgs/SectorScan` (`/molly/perception/sector_scan`), `visualization_msgs/MarkerArray` (`/molly/perception/markers`) |

## Build
```bash
cd ~/molly_ws
colcon build --packages-select molly_navigation_msgs livox_ros_driver2 molly_perception
source install/setup.bash
```

## Run
```bash
# 1) driver Livox, mode CustomMsg
ros2 launch livox_ros_driver2 msg_MID360.launch.py   # atau launch file driver yg xfer_format:=1

# 2) perception node ini
ros2 launch molly_perception perception.launch.py
```

Cek di RViz: tambah `MarkerArray` display, topic `/molly/perception/markers`,
fixed frame `base_link`. Ray + titik warna merah (dekat) -> kuning -> hijau
(jauh).

## Tuning
- **Lantai kedetek jadi obstacle** -> naikin `obstacle_z_min`.
- **Objek rendah (kaki manusia, kabel) lolos** -> turunin `obstacle_z_min`.
- **Mounting Livox berubah** -> update `lidar_offset_x/y/z`, `lidar_yaw_deg`.
- **Markers berat di RViz / gak butuh viz saat produksi** -> `publish_markers: false`.
- **Mau resolusi lebih halus dari 32** -> tinggal ganti `num_sectors`, semua
  downstream (`obstacle_avoidance_node`) generic terhadap jumlah sektor.
- **Mau lihat 360° lagi (bukan cuma depan)** -> ganti `fov_deg: 360.0`.

## Known limitations
- Floor removal cuma height-band filter (`obstacle_z_min/max`), asumsi lantai flat.
- Ekstrinsik lidar->base_link statis (bukan TF live).
- Livox-specific: kalau ganti sensor non-Livox, node ini gak jalan, harus
  versi PointCloud2.
- `xfer_format` di CustomMsg mode gak selalu identik behavior-nya dengan mode
  PointCloud2 untuk multi-lidar (`lidar_id` per titik) -- kalau nanti ada 2+
  Livox sekaligus, cek `msg.lidar_id` per titik, filter kalau perlu (saat ini
  belum di-handle, single-lidar assumption).

---
title: Tuning Guide
header: Pashupati Tuning Guide
subtitle: What the parameters do, and how to tune a Genisom L1W for a site
version: "2.0"
date: 2026-10-29
audience: >-
  For the technician who sets up and tunes a Pashupati robot. You need an SSH shell on the robot
  computer and some ROS 2 basics.
note: >-
  Operating the robot is described in the Pashupati User Manual. Pashupati 3 runs on the Dharma
  0.2 stack: every Dharma parameter is typed (unit, range, description) and listed by
  `ros2 param describe`. The parameter reference at the end of this guide is generated from
  Pashupati's parameter files. Units: m, s, m/s, rad/s, deg; 0-1 means a fraction.
revisions:
  - ["1.0", "2026-10-16", "First version: Pashupati with the Genisom L1W"]
  - ["2.0", "2026-10-29", "Pashupati 3 on Dharma 0.2: robot profile, stack file, sites, new parameter names"]
---

# How tuning works

## Where a value comes from

Every node starts with the defaults of its package. Pashupati then applies, in this order (later
wins):

| Layer         | Where                                                 | Changed by                                                                                                |
| ------------- | ----------------------------------------------------- | --------------------------------------------------------------------------------------------------------- |
| Robot profile | `src/pashupati/config/robot.yaml` (or `ROBOT_PARAMS`) | a Pashupati or robot update                                                                               |
| Stack file    | `src/pashupati/config/stack.yaml`                     | a Pashupati update                                                                                        |
| Site file     | `/etc/pashupati/site.yaml` (`SITE_CONFIG`)            | you, for this robot and site                                                                              |
| Launch        | non-empty values of `/etc/pashupati/pashupati.env`    | you (chapter 9)                                                                                          |
| Live changes  | in the running node                                   | handheld, `ros2 param set`, RViz; lost at a restart, except the handheld's inspection and people settings |

The **robot profile** (`robot.*`) is the one description of the robot that every node reads:
kinematics, base height, footprint, speed limits and braking (chapter 2). The **stack file**
holds only what Pashupati changes from the Dharma defaults; most values tuned on the robot in
Pashupati 1 and 2 became the Dharma defaults.

Never change the repository on a robot: the next `git pull` conflicts with your change. Put
every change in the site file. The robot driver (`l1w_driver_node`) is not a Pashupati node; its
settings live in the robot package (section 2.3).

## Change a value for good

1. Open the site file: `sudo nano /etc/pashupati/site.yaml`.
2. Find the node's section, or add one like the example below. Indent with spaces, two per
   level, never with tabs.
3. Restart the stack: `sudo systemctl restart pashupati`.
4. Check the start of the log for warnings: `journalctl -u pashupati -n 100 | grep -i warn`.

```yaml
path_follower_node:
  ros__parameters:
    speed:
      cruise: 0.35
```

A misspelt **node** name is reported at start (_no node is called "…"_). A misspelt
**parameter** is reported by the node (_undeclared parameter_), and a value out of its range
stops the node with the reason. A file that is not valid YAML stops the start, so the robot never
runs on defaults by accident.

## Try a value live

```bash
pashu                                                    # ROS shell of the stack
ros2 param describe /path_follower_node speed.cruise     # unit, range, live or read-only
ros2 param set /path_follower_node speed.cruise 0.3
ros2 param dump /path_follower_node                      # every value in use
```

Safety values (limits, margins, timeouts) are **read-only at runtime**: `ros2 param set` refuses
them; change them in the site file and restart. The **Tuning** panel of RViz (`dharma_rviz`)
edits the live parameters of any node; **Save YAML** writes them for the site file. RViz runs on
a laptop with ROS 2 Humble on the robot's network: set `ROS_LOCALHOST_ONLY=0` in the env file,
restart, and use the robot's `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` on the laptop.

!!! danger "Set it back"
    With `ROS_LOCALHOST_ONLY=0` anyone on the robot's network can send motion commands to the
    robot. Set it back to `1` and restart when you are done.

## What the handheld changes

| On the handheld                         | Parameter                                                                                                                                    | After a restart     |
| --------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------- | ------------------- |
| Patrol speed slider                     | `path_follower_node` `speed.cruise`, from `teleop_udp_node` `patrol_speed_min` to `patrol_speed_max` (and never above `robot.limits.max_vx`) | the site file value |
| Inspect the floor                       | `anomaly_detector_node` `enabled`                                                                                                            | kept                |
| Pace: Continuous, Interval, Stop points | `interval_sec`: 0, the interval, -1                                                                                                          | kept                |
| Model                                   | `model`                                                                                                                                      | kept                |
| Finding types                           | `categories`, `alarm_categories`, `category_descriptions`                                                                                    | kept                |
| Count people                            | `person_detector_node` `enabled`                                                                                                             | kept                |
| Speed 1, 2, 3                           | 30, 60, 100 % of `teleop_udp_node` `max_vx`, `max_vy`, `max_wz` (capped by `robot.limits`)                                                   | a handheld setting  |
| Inspection at a stop point              | the dwell time stored in the route with each **Point**                                                                                       | in the route        |

_Kept_ values are stored in `~/.pashupati/inspection.json` and `~/.pashupati/people.json` and
**win over the site file**. To go back to the site file's inspection settings, stop the service,
delete `inspection.json`, and start it again.

## Tuning a new site, step by step

1. **The robot, once:** the robot profile and the stick channels (chapter 2).
2. **Braking:** measure it and test the protective field (chapter 3).
3. **The site:** map it, print, place and record the AprilTags (chapter 6).
4. **First route:** teach a short route and patrol it at 0.3 m/s (chapter 4).
5. **Obstacles:** a cardboard box on the route, then a person walking past (chapter 5).
6. **Full speed:** raise the patrol speed step by step, one full lap per step.
7. **Inspection and people:** pace, model, finding types, people counting (chapter 7).
8. **Save:** copy every value that works into the site file, restart, drive one more lap.

Try a change on the simulator first (`ros2 launch pashupati sim.launch.py rviz:=true`): it runs
the same stack with the same files.

## Work safely

- Tune in a closed area first, never in a crowd.
- Keep the ZSBot remote in reach, and someone on the handheld's STOP.
- Change one thing at a time. Write down the old and the new value.
- Start low: speeds and accelerations below what you need, then raise them step by step.
- Repeat the protective field test (section 3.3) after every change to the robot profile.

# The Genisom L1W

## The robot profile

`src/pashupati/config/robot.yaml` describes a ~0.80 × 0.56 m quadruped, field-tested on the L1W.
A robot package can ship its own profile (`ROBOT_PARAMS`); to change one value for one robot,
override it in the site file under `/**:` (every node reads it).

| Parameter                                       | L1W              | What it is                                                                                                                                  |
| ----------------------------------------------- | ---------------- | ------------------------------------------------------------------------------------------------------------------------------------------- |
| `robot.kinematics`                              | omni             | the L1W strafes; `diff` for a robot that cannot (then `max_vy` 0)                                                                           |
| `robot.base_height`                             | 0.35             | m, `base_link` above the floor when standing (the floor for the LiDAR and for placing people)                                               |
| `robot.footprint.front`, `.rear`, `.half_width` | 0.40, 0.40, 0.28 | m from `base_link` to the body's edges, legs included: the protective field, the planner, path shift and the LiDAR's self filter all use it |
| `robot.limits.max_vx`, `max_vy`, `max_wz`       | 1.0, 0.5, 1.5    | nothing may command more, in AUTO or REMOTE                                                                                                 |
| `robot.limits.max_accel`, `max_yaw_accel`       | 0.5, 1.5         | m/s², rad/s²                                                                                                                                |
| `robot.limits.max_decel`                        | 0.6              | m/s², braking the robot reliably achieves: **measure it** (section 3.2)                                                                     |
| `robot.limits.reaction_time`                    | 0.25             | s, from the command to the robot actually braking                                                                                           |

To measure the height of `base_link`, stand the robot on a flat floor, measure the height of the
LiDAR above the floor, and subtract the LiDAR's height in the body:
`ros2 run tf2_ros tf2_echo base_link livox_frame` (the z value). Measure the footprint again when
the robot carries a payload or a cover that sticks out.

```yaml
/**: # in the site file: a heavier L1W with a cover
  ros__parameters:
    robot:
      footprint: { front: 0.45, rear: 0.40, half_width: 0.30 }
      limits: { max_decel: 0.5 }
```

## Speed limits

| Parameter                                                        | L1W           | Limits                                                  |
| ---------------------------------------------------------------- | ------------- | ------------------------------------------------------- |
| `robot.limits.max_vx`, `max_vy`, `max_wz`                        | 1.0, 0.5, 1.5 | every command: the control mux clamps to them           |
| `teleop_udp_node` `max_vx`, `max_vy`, `max_wz`                   | 1.0, 0.5, 1.5 | the handheld at speed level 3 (never above the profile) |
| `path_follower_node` `speed.cruise`                              | 0.4           | the patrol speed (the handheld's slider changes it)     |
| `l1w_driver_node` `stick_max_vx`, `stick_max_vy`, `stick_max_wz` | 1.0, 0.5, 1.5 | the command that means a full stick (next section)      |

## Stick channels and speed scaling

The L1W takes motion as joystick values, like its own remote. `l1w_driver_node` turns each
speed command into a stick position: a command of `stick_max_vx` is a full stick forward. Its
settings are in the robot package,
`src/xtras/robots/genisom_l1w_ros2/l1w_driver/config/l1w_driver.yaml`. Edit them there and
restart the service; they are not live.

| Parameter                                      | L1W           | Meaning                                                            |
| ---------------------------------------------- | ------------- | ------------------------------------------------------------------ |
| `stick_max_vx`, `stick_max_vy`, `stick_max_wz` | 1.0, 0.5, 1.5 | speed that maps to full stick deflection                           |
| `stick_yaw_axis`, `stick_lateral_axis`         | 2, 1          | stick channels for turning and walking sideways                    |
| `stick_invert_yaw`, `stick_invert_lateral`     | false         | true when the robot turns or walks sideways the wrong way          |
| `cmd_timeout`                                  | 0.5           | s; without a new command the stick returns to zero                 |
| `auto_control_right`                           | true          | take motion control from the ZSBot remote when the driver connects |

**Check the directions once** (REMOTE, speed level 1): left stick to the left must walk the
robot to its left; right stick to the left must turn it to the left. If one is wrong, set its
`stick_invert_*` to true.

**Check the speed scale:** the speed the L1W reaches at full stick depends on its speed gear
(_Slow_, _Normal_, _Fast speed_ in the robot actions). Patrol a straight route and compare the
patrol speed with the measured speed: `ros2 topic echo /l1w/odom --field twist.twist.linear.x`.
If the robot is slower or faster, set `stick_max_vx` to its value × measured speed ÷ patrol
speed, and scale `stick_max_vy` and `stick_max_wz` the same way. Always patrol in the gear you
checked.

# Protective field

## How it works

`control_mux_node` is the only node that commands the robot. It limits every command to a speed
at which the robot can still stop a margin short of the nearest obstacle in the path it is
about to sweep, in every direction (the LiDAR's 360° safety scan), from the robot profile's
`max_decel` and `reaction_time` and an assumed walking speed of a person toward the robot. The
Dharma guide _Safety_ gives the formula.

The field always works in AUTO. In REMOTE the operator watches the robot: Pashupati drives
REMOTE without the field (`field.in_remote: false`, a field-tested decision); the speed limits
and the e-stop still apply. Without a fresh scan the robot does not drive itself at all.

| `control_mux_node` parameter            | Default               | What it does                                                                             |
| --------------------------------------- | --------------------- | ---------------------------------------------------------------------------------------- |
| `field.in_remote`                       | false                 | true: limit REMOTE too (with `field.remote_margin`), e.g. for operators new to the robot |
| `field.margin`                          | 0.15                  | m, gap left to the obstacle in AUTO                                                      |
| `field.remote_margin`                   | 0.10                  | m, the same in REMOTE                                                                    |
| `field.lateral_margin`                  | 0.10                  | m, extra width of the swept path on each side                                            |
| `field.human_speed`                     | 0.5                   | m/s, a person walking toward the robot                                                   |
| `field.min_speed`                       | 0.02                  | m/s, a slower allowed speed stops instead                                                |
| `field.remote_degraded_speed`           | 0.15                  | m/s, the REMOTE cap while the scan is stale or blind                                     |
| `field.scan_timeout_sec`                | 0.5                   | s; an older scan stops AUTO and caps REMOTE                                              |
| `use_health_limit`                      | true                  | scale every command by the health monitor's speed limit (section 8.5)                      |
| `remote_sources`                        | [handheld, dashboard] | who may drive in REMOTE (`control/remote/<source>/cmd_vel`)                              |
| `remote_timeout_sec`, `nav_timeout_sec` | 0.3, 0.5              | s; a silent source gets zero speed                                                       |

A lower `max_decel`, a longer `reaction_time` or a bigger margin make the robot slow down earlier
and stop further from obstacles. Too low, and it crawls in narrow aisles.

## Measure the braking

1. Teach a straight route of at least 8 m, with free space beyond it.
2. Set the patrol speed on the handheld to the fastest you will patrol at, for example 0.6.
3. Put a tape mark on the floor in the middle of the route.
4. Start the patrol. Press **STOP** when the front of the robot passes the mark.
5. Measure the distance _d_ from the mark to the front of the stopped robot.
6. max*decel = speed² ÷ (2 × (\_d* − speed × 0.25)).
7. Repeat three times. Use the smallest result as `robot.limits.max_decel`.

Your own reaction is part of _d_, so the result errs on the safe side. At 0.6 m/s:

| Stopping distance _d_    | 0.40 m | 0.50 m | 0.60 m | 0.80 m | 1.00 m |
| ------------------------ | ------ | ------ | ------ | ------ | ------ |
| `robot.limits.max_decel` | 0.72   | 0.51   | 0.40   | 0.28   | 0.21   |

Measure on every new kind of floor: a polished floor brakes worse than a rough one.

## Test it

1. In the site file set `control_mux_node` `field.in_remote: true` and restart.
2. Put a cardboard box in an open area.
3. In REMOTE at speed level 1, drive straight at the box. The robot must slow down and stop
   about `field.remote_margin` (10 cm) before it. Try level 2, then from the side and backwards.
4. Set `field.in_remote` back to false (or keep it on) and restart.

If the robot gets closer than the margin, lower `robot.limits.max_decel` or raise
`robot.limits.reaction_time`. Never test with a person in front of the robot.

# The patrol

## Speed and motion

All in `path_follower_node`; the speed values are live. _Pashupati_ is the value with the stack
file (the Dharma default where Pashupati sets nothing).

| Parameter                    | Pashupati | What it does · when to change                                     |
| ---------------------------- | --------- | ----------------------------------------------------------------- |
| `speed.cruise`               | 0.4       | m/s, the patrol speed; keep it low in crowds                      |
| `speed.min_speed`            | 0.1       | m/s, the slowest the speed profile goes                           |
| `speed.lateral_accel`        | 0.3       | m/s², the speed in curves: lower is slower in bends               |
| `speed.accel`, `speed.decel` | 0.5, 0.3  | m/s², starting and braking into stops (Pashupati: 0.5 to start)   |
| `speed.approach_speed`       | 0.15      | m/s when arriving at stop points and the route end                |
| `speed.smoothing`            | 0.4       | 0-1, lower is smoother and slower to react                        |
| `controller`                 | holonomic | x, y and yaw together; strafes, keeps its heading along the route |

## Stop points

At each stop point the robot converges (`approach.*`), calls the inspection hook, waits for its
answer, dwells for the route's dwell time, optionally backs up, and walks on.

| Parameter                                                   | Pashupati                          | What it does                                                                                                           |
| ----------------------------------------------------------- | ---------------------------------- | ---------------------------------------------------------------------------------------------------------------------- |
| `approach.position_tolerance`, `approach.yaw_tolerance_deg` | 0.05, 3.0                          | m, deg: how close the robot converges                                                                                  |
| `approach.min_speed`                                        | 0.04                               | m/s, beats the robot's joystick deadband                                                                               |
| `approach.timeout_sec`                                      | 20                                 | s, then it dwells where it is                                                                                          |
| `hooks.inspection`                                          | [/anomaly_detector_node/check_now] | services called at each stop; a hook whose node does not run is skipped, so `INSPECTION=false` never blocks the patrol |
| `hooks.inspection_timeout_sec`                              | 20                                 | s; VLM inference on the Jetson takes a few seconds                                                                     |
| `inspection.reverse_distance_m`                             | 0                                  | m; > 0 backs up this far after the inspection (the LiDAR sectors look 360° for it)                                     |
| `loop_route`                                                | true                               | a closed route starts its next lap                                                                                     |
| `low_battery`, `critical_battery`                           | 0.2, 0.1                           | 0-1: below `low_battery` the patrol ends its lap; below `critical_battery` it pauses                                   |

## Leaving and joining the route

| Parameter                                      | Pashupati | What it does                                                     |
| ---------------------------------------------- | --------- | ---------------------------------------------------------------- |
| `join.max_distance`                            | 3.0       | m; a robot started beside the route walks onto it from here      |
| `safety.off_path_slow`, `safety.off_path_stop` | 0.6, 1.5  | m of cross-track error: slow down, stop (_off the route_)        |
| `safety.blocked_timeout_sec`                   | 30        | s standing in front of an obstacle before _Blocked_ (Dharma: 10) |
| `require_localization`                         | true      | **Start** is refused until the robot is localized                |

# Obstacles

## Path shift

The robot passes an obstacle by sliding sideways, never further than `path_shift.max_offset`
from the route, and stops short when there is no gap.

| `path_follower_node` parameter                       | Default    | What it does                                         |
| ---------------------------------------------------- | ---------- | ---------------------------------------------------- |
| `avoidance_mode`                                     | path_shift | or a reactive policy of `obstacle_avoidance_node`    |
| `path_shift.max_offset`                              | 0.8        | m, furthest from the route                           |
| `path_shift.clearance`                               | 0.10       | m, gap kept between the robot's side and an obstacle |
| `path_shift.stop_margin`                             | 0.40       | m, stop this far before an obstacle                  |
| `path_shift.switch_cost`, `path_shift.switch_margin` | 0.5, 0.05  | keeps its side around an obstacle                    |

## What the LiDAR sees

`lidar_sector_node` turns the LiDAR into sectors (front, for the avoidance) and the 360° safety
scan (for the protective field). The band between `filter.height_min` and `filter.height_max`
above the floor (the floor is `robot.base_height` below `base_link`) counts as obstacles; the
robot's own footprint, grown by `filter.self_margin`, is ignored.

| Parameter                            | Pashupati | What it does · when to change                                                      |
| ------------------------------------ | --------- | ---------------------------------------------------------------------------------- |
| `filter.height_min`                  | 0.10      | m above the floor; lower sees low objects, too low sees the floor on slopes        |
| `filter.height_max`                  | 1.55      | m above the floor (the L1W's top plus margin)                                      |
| `filter.self_margin`                 | 0.05      | m around the footprint: _Slowed_ or _Blocked_ with nothing around, widen it 2-3 cm |
| `front.num_sectors`, `front.fov_deg` | 64, 360   | the avoidance also sees behind (needed by the back-up after an inspection)         |

# Sites, maps and localization

## Sites

A **site** is one place: its occupancy map, its AprilTag poses and its taught routes, in
`SITES_DIR/<site>/` (`~/.pashupati/sites`). `SITE` in the env file picks the site at start (empty:
the last one). `MODE=teach` maps the site from the start; drive slowly through the whole place in
REMOTE, then save the map (`/mapping/save`). Taught routes go to `<site>/routes/`, routes from
NETRA to `<site>/netra/`.

## AprilTags

The tag detector (`apriltag`, from dharma_perception) must know every tag on the site's walls;
localization uses the tags' frames:

```yaml
apriltag:
  ros__parameters:
    tag:
      ids: [2, 4]
      frames: [base_map, charging_dock]
      sizes: [0.15, 0.15] # MEASURE the printed black square
localization_node:
  ros__parameters:
    tag:
      frames: [base_map, charging_dock]
```

Place tags at about the camera's height along the route, where the robot passes 1-2 m from them.
Record them while localized: `/localization/record_tags` (the handheld's **Tags**) averages the
visible tags over `tag.record_duration_sec` and writes the site's `tags.json`.

| `localization_node` parameter                  | What it does                                             |
| ---------------------------------------------- | -------------------------------------------------------- |
| `tag.auto_localize`                            | localize from the first known tag after start-up         |
| `tag.auto_correct`                             | correct the pose from tags while patrolling              |
| `tag.max_distance_m`, `tag.max_view_angle_deg` | tags further away or seen at a steeper angle are ignored |
| `estimator.sigma_xy_per_sqrt_m`                | how fast the position is assumed to drift between tags   |

## Odometry

FAST-LIO2 (`fastlio_mapping_node`) starts once the robot stands still. Pashupati keeps the
on-line extrinsic estimation that was field-tested on the L1W (`mapping.extrinsic_est_en: true`)
and lets `odometry_gate` accept a standing quadruped's lean (`max_tilt_deg: 8`).

# Inspection and people

## Pace and queue

`anomaly_detector_node` sends camera images to a vision-language model (Ollama, on the robot) and
gets back one finding type or _none_.

| Parameter           | Default                         | What it does · when to change                                                        |
| ------------------- | ------------------------------- | ------------------------------------------------------------------------------------ |
| `enabled`           | true                            | live; off also unloads the model from memory and answers the stop-point hook at once |
| `interval_sec`      | 0                               | live: 0 continuous, > 0 one image every N s, < 0 only at stop points                 |
| `queue_max`         | 50                              | interval pace: images waiting for the model; when full, the oldest is dropped        |
| `queue_min_change`  | 12                              | grey levels the picture must change to be queued again; 0 = every image              |
| `queue_dir`         | `~/.pashupati/inspection_queue` | waiting images survive a restart; "" = memory only                                   |
| `cooldown_sec`      | 60                              | s; continuous and interval pace do not report the same type again within this time   |
| `max_image_age_sec` | 2.0                             | s; older camera images are not inspected                                             |
| `max_image_width`   | 640                             | px; images are shrunk first                                                          |
| `ollama_url`        | `http://127.0.0.1:11434`        | the Ollama server (`OLLAMA_URL` in the env file)                                     |

## Model and memory

| Parameter     | Default              | What it does                                                    |
| ------------- | -------------------- | --------------------------------------------------------------- |
| `model`       | qwen3-vl:2b-instruct | live; the model in use                                          |
| `models`      | three models         | offered on the handheld (plus any other vision model in Ollama) |
| `keep_alive`  | "-1"                 | keep the model loaded: "-1" forever, or e.g. "30m" (quoted)     |
| `num_ctx`     | 2048                 | tokens of context; many long descriptions: 4096 (more memory)   |
| `timeout_sec` | 30                   | s, longest wait for one answer                                  |
| `use_schema`  | true                 | structured answers; false only for models that cannot do them   |

A model must be pulled before it can be used (`ollama pull <model>`, or
`scripts/setup.sh install ollama` with `VLM_EXTRA_MODELS`). `setup.sh install ollama` also turns
off Ollama's prompt cache, which otherwise grows to several GB of RAM.

## Finding types

| Parameter               | Default               | What it does                                                   |
| ----------------------- | --------------------- | -------------------------------------------------------------- |
| `categories`            | [trash, spill, stain] | live; the types the model chooses from (plus _none_), up to 12 |
| `alarm_categories`      | [trash, spill]        | live; only these are reported to NETRA                         |
| `category_descriptions` | [""]                  | live; own text per type, `"name: what to look for"`            |

`trash`, `spill`, `stain`, `floor_damage` and `fallen_person` have built-in descriptions. Good
descriptions are short and concrete, name what is visible, and say what is **not** a finding
(_not shadows, reflections or floor patterns_). `fleet_uplink_node` sends findings with at least
`anomaly_min_confidence` (0.5).

## People counting

`person_detector_node` finds people with YOLO11n (`PEOPLE=true`; the handheld switches it on and
off). It places people on the floor `robot.base_height` below `base_link`.

| Parameter    | Default | What it does · when to change                               |
| ------------ | ------- | ----------------------------------------------------------- |
| `enabled`    | true    | live                                                        |
| `confidence` | 0.4     | live, 0-1; boxes on things that are not people: raise it    |
| `rate_hz`    | 0       | live; detections per second, 0 = 5 on the GPU, 2 on the CPU |
| `max_range`  | 12      | m; people further away are not placed on the map            |
| `backend`    | auto    | `onnxruntime`, `opencv` or `ultralytics`                    |

NETRA's crowd map uses `fleet_uplink_node` `crowd_cell_size` (1 m) and `crowd_window_sec` (60).

# Handheld, camera, video, NETRA and health

## Handheld link

`teleop_udp_node`, read at start:

| Parameter                                     | Default                              | What it does                                                     |
| --------------------------------------------- | ------------------------------------ | ---------------------------------------------------------------- |
| `token`                                       | ""                                   | shared secret; the same in the handheld. **Always set it**       |
| `robot_name`                                  | ""                                   | name on the handheld's start screen (empty: the computer's name) |
| `port`                                        | 9870                                 | UDP port of the link and of robot discovery                      |
| `client_timeout_sec`                          | 2.0                                  | s; a silent handheld is dropped                                  |
| `robot_commands`                              | [stand_up, sit_down, emergency_stop] | robot actions on the handheld: `/<ROBOT_NS>/<name>` services     |
| `patrol_speed_min`, `patrol_speed_max`        | 0.1, 0.6                             | m/s range of the patrol speed slider                             |
| `lidar_hz`, `lidar_voxel`, `lidar_max_points` | 2, 0.1, 3000                         | LiDAR points on the handheld's map                               |

## Camera source

`CAMERA=driver` uses the camera of the robot package (`CAMERA_TOPICS`, `/camera/camera/color`).
`CAMERA=rtsp` starts `rtsp_camera` for an IP camera at `CAMERA_URL`, published on the same topics:
`calibration_file` (a camera_calibration YAML, see `config/calibration.example.yaml`) rectifies
the image; without one, `default_hfov_deg` (90) gives an approximate `camera_info`, enough for
inspection and video but not for exact tag poses.

## Video

The handheld gets H.264 over WebRTC from `video_stream_node`, started while it watches:
`bitrate_kbps` (1500; weak Wi-Fi: 800), `fps` (15), `max_width` (1280), `keyframe_sec` (1.0),
`encoder` (auto). Without WebRTC (`WEBRTC=false` or no MediaMTX) the handheld gets JPEG frames:
`teleop_udp_node` `camera_fps` (8), `camera_jpeg_quality` (60), `camera_max_width` (640).

## NETRA uplink

`fleet_uplink_node`: `server_url` (`CC_URL`), `token` (`CC_TOKEN`), `robot_id` (`ROBOT_ID` wins),
`telemetry_hz` (1.0), `upload_period_sec` (5.0), `verify_tls` (true; keep it on), `spool_dir`
(`~/.pashupati/spool`: findings and routes wait there while NETRA is unreachable).

## Health monitor

`health_monitor_node` publishes `/health/status` (level, issues, speed limit), shown on the
handheld and in NETRA. `control_mux_node` scales every command by its speed limit.

| Parameter                                                                   | Pashupati                                                        | Issue when                                        |
| --------------------------------------------------------------------------- | ---------------------------------------------------------------- | ------------------------------------------------- |
| `sensors.topics`                                                            | LiDAR 5, odometry 8, sectors 5, camera_info 5, robot odometry 10 | a topic runs slower (`"topic:Hz[:max_age]"`)      |
| `degradation.rules`                                                         | LiDAR, odometry or TF stale: speed × 0.5                         | a degraded part slows the robot                   |
| `nodes.expected`                                                            | empty                                                            | a listed node is missing (add the driver's nodes) |
| `system.max_temp_c`, `system.max_memory_percent`, `system.min_disk_free_gb` | 85, 92, 5                                                        | the computer is hot, full or out of disk          |
| `startup_grace_sec`                                                         | 30                                                               | s after start without missing-node issues         |

# The env file

`/etc/pashupati/pashupati.env` says what runs. Restart the service after a change. Quote values
with spaces, and never put a comment after a value.

| Key                                       | Genisom L1W                                               | Meaning                                                    |
| ----------------------------------------- | --------------------------------------------------------- | ---------------------------------------------------------- |
| `ROBOT_ID`                                | e.g. `l1w-01`                                             | the robot's name in NETRA                                  |
| `ROBOT_DRIVER`                            | `l1w_bringup`                                             | robot package started with the stack (was `ROBOT_BRINGUP`) |
| `ROBOT_PARAMS`                            | empty                                                     | robot profile; empty: `config/robot.yaml`                  |
| `ROBOT_NS`                                | `l1w`                                                     | namespace of the driver's topics and services              |
| `ROBOT_PRESTART`                          | `"/usr/local/bin/robot-launch stop driver-livox forward"` | stops the vendor's own LiDAR driver and motion stack first |
| `ROBOT_SETUP`, `ROBOT_IP`                 | the robot package's installer, `192.168.234.1`            | the ZSBot SDK, the network check                           |
| `MODE`                                    | `patrol`                                                  | `teach`: map `SITE` from the start                         |
| `ODOMETRY`                                | `fast_lio`                                                | or `leg_kilo`                                              |
| `SITE`, `SITES_DIR`                       | the mall's site, `~/.pashupati/sites`                     | the site at start (was `ROUTE` and `TAGS`)                 |
| `CAMERA`, `CAMERA_URL`, `CAMERA_TOPICS`   | `driver`, empty, `/camera/camera/color`                   | section 8.2                                                |
| `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`     | `0`, `rmw_fastrtps_cpp`                                   | Pashupati's own DDS; the vendor stack uses another         |
| `ROS_LOCALHOST_ONLY`                      | `1`                                                       | `0` only for RViz from a laptop (section 1.3)              |
| `PEOPLE`, `INSPECTION`, `OLLAMA_URL`      | `true`, `true`, empty                                     | people counting, the VLM inspection                        |
| `SITE_CONFIG`                             | `/etc/pashupati/site.yaml`                                | the site file                                              |
| `WEBRTC`, `MEDIAMTX`                      | `true`, empty                                             | WebRTC video for the handheld                              |
| `CC_URL`, `CC_TOKEN`                      | NETRA's address, the robot's token                        | NETRA uplink; empty: findings are kept on the robot        |
| `TIME_SYNC_WAIT`                          | `60`                                                      | s to wait at boot for the network clock                    |
| `OPENBLAS_NUM_THREADS`, `OMP_NUM_THREADS` | `1`                                                       | keep at 1: more threads waste CPU                          |

`scripts/setup.sh check` reports a missing key, or a line that breaks the rules above.

# Symptoms and fixes

| Symptom                                         | Look at                                                            | Try                                               |
| ----------------------------------------------- | ------------------------------------------------------------------ | ------------------------------------------------- |
| Weaves left and right on straight parts         | `speed.smoothing`, the `holonomic.*` section                       | smoothing 0.3                                     |
| Too slow in gentle curves                       | `speed.lateral_accel`                                              | 0.4                                               |
| Jerky starts and stops                          | `speed.accel`, `speed.decel`                                       | 0.3, 0.2                                          |
| Walks slower or faster than the patrol speed    | `stick_max_*` (section 2.3), speed gear                            | rescale; patrol in the checked gear               |
| Turns or walks sideways the wrong way           | `stick_invert_yaw`, `stick_invert_lateral`                         | true                                              |
| Stops just short of a stop point and waits 20 s | `approach.min_speed`, `approach.position_tolerance`                | 0.06, 0.08                                        |
| Leaves stop points too late                     | dwell time of the route, `hooks.inspection_timeout_sec`, pace      | shorter dwell, faster model                       |
| _Slowed_ or _Blocked_ with nothing around       | `filter.self_margin`, `filter.height_min`, floor slopes            | margin +0.02; height_min +0.03                    |
| Ignores low objects (bags, boxes)               | `filter.height_min`                                                | lower 0.03 at a time, check the floor is not seen |
| Stops far before obstacles                      | `path_shift.stop_margin`, `field.margin`, `robot.limits.max_decel` | small steps; repeat the test of section 3.3       |
| Gets too close to obstacles                     | `robot.limits.max_decel`, `robot.limits.reaction_time`             | measure the braking again (section 3.2)           |
| Does not pass gaps that look wide enough        | `path_shift.clearance`, `path_shift.max_offset`, the footprint     | clearance 0.05, larger offset                     |
| _Blocked_ too soon or too late                  | `safety.blocked_timeout_sec`                                       | to taste                                          |
| **Start** refused: robot too far from the route | `join.max_distance`                                                | drive closer, or raise it                         |
| **Start** refused: not localized                | a tag in view, `/localization/start`                               | **Localize** on the handheld                      |
| _No loc_: tag not found                         | `apriltag` `tag.sizes`, `tag.ids`, light, `detector.decimate`      | measure the tag, decimate 1.0                     |
| Same trash reported again and again             | `cooldown_sec`                                                     | 120-300                                           |
| False findings                                  | type description, `alarm_categories`, `anomaly_min_confidence`     | describe what is not a finding; 0.6-0.7           |
| Inspection queue always full                    | `interval_sec`, `model`, `max_image_width`                         | longer interval, smaller model                    |
| Robot slow everywhere, health _warn_            | `/health/status` issues, `degradation.rules`                       | fix the stale sensor                              |
| Video stutters                                  | `bitrate_kbps`, `fps`                                              | 800, 10                                           |
| Handheld map lags                               | `lidar_hz`, `lidar_voxel`, `lidar_max_points`                      | 1, 0.2, 1500                                      |
| Findings missing in NETRA                       | `CC_URL`, `CC_TOKEN`, `alarm_categories`, `anomaly_min_confidence` | the log of `fleet_uplink_node`                    |

# Site file for the Genisom L1W

A starting point for `/etc/pashupati/site.yaml` (the repository's
`src/pashupati/config/site.example.yaml` is the same). The robot itself is the robot profile;
the site file holds this robot's token and actions and this site's values.

```yaml
teleop_udp_node:
  ros__parameters:
    token: "change-me" # the same in the handheld
    robot_commands:
      [
        stand_up,
        sit_down,
        balance_stand_mode,
        move_mode,
        lock_mode,
        slow_speed,
        normal_speed,
        fast_speed,
        sdk_control_right,
        emergency_stop,
      ]

apriltag: # the tags on this site's walls
  ros__parameters:
    tag:
      ids: [2, 4]
      frames: [base_map, charging_dock]
      sizes: [0.15, 0.15] # MEASURE the printed square

path_follower_node:
  ros__parameters:
    speed:
      cruise: 0.35 # patrol speed for this site
    low_battery: 0.25

fleet_uplink_node:
  ros__parameters:
    robot_id: l1w-01 # ROBOT_ID in the env file wins
```

Each node may appear only once in the file: add more parameters under its existing heading.
After a restart, `ros2 param dump /control_mux_node` (and so on) shows the values in use.

# Parameter reference

The parameters Pashupati sets, read from its parameter files of this version (the robot profile,
the stack file, and the defaults of the Pashupati packages). _Meaning_ is the comment in the
file. Every other Dharma parameter keeps its default: `ros2 param describe /<node> <name>`, and
the Dharma manual lists them all.

{{ parameter_reference }}

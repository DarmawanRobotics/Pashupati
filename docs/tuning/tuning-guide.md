---
title: Tuning Guide
header: Pashupati Tuning Guide
subtitle: What every parameter does, and how to tune a Genisom L1W for a site
version: "1.0"
date: 2026-10-16
audience: >-
  For the technician who sets up and tunes a Pashupati robot. You need an SSH shell on the robot
  computer and some ROS 2 basics.
note: >-
  Operating the robot is described in the Pashupati User Manual. The parameter reference at the
  end of this guide is generated from the parameter files of this Pashupati version, so it lists
  every parameter with its default value. Units: m, s, m/s, rad/s, deg; 0-1 means a fraction.
revisions:
  - ["1.0", "2026-10-16", "First version: Pashupati with the Genisom L1W"]
---

# How tuning works

## Where a value comes from

Every Pashupati node starts with the defaults of its package. The site file is applied on top,
and live changes on top of that.

| Layer | Where | Changed by | After a restart |
|---|---|---|---|
| Package defaults | `src/<package>/config/*.yaml` | a Pashupati update | used |
| Site file | `/etc/pashupati/site.yaml` | you, for this robot and site | used |
| Live changes | in the running node | handheld, `ros2 param set`, RViz | lost, except the handheld's inspection and people settings |

Never change the package defaults on a robot: the next `git pull` conflicts with your change.
Put every change in the site file. The env file `/etc/pashupati/pashupati.env` is not for
tuning: it says which robot package runs, which extras start and where NETRA is (chapter 13).

The robot driver (`l1w_driver_node`) is not a Pashupati node. Its settings live in the robot
package (section 2.4).

## Change a value for good

1. Open the site file: `sudo nano /etc/pashupati/site.yaml`.
2. Find the node's section, or add one like the example below. Indent with spaces, two per
   level, never with tabs.
3. Restart the stack: `sudo systemctl restart pashupati`.
4. Check the start of the log for warnings: `journalctl -u pashupati -n 100 | grep -i warn`.

```yaml
path_follower_node:
  ros__parameters:
    target_linear_velocity: 0.35
```

A misspelt **node** name is reported at start: *no node is called "…" (typo?)*. A file that is
not valid YAML stops the start, so the robot never runs on defaults by accident. A misspelt
**parameter** name is silently ignored: check that your value arrived with `ros2 param get`.

## Try a value live

```bash
pashu                                   # ROS shell of the stack
ros2 param get /path_follower_node target_linear_velocity
ros2 param set /path_follower_node target_linear_velocity 0.3
ros2 param dump /path_follower_node     # every value in use
```

A live change applies at once and is lost at the next restart. When a value works, copy it to
the site file. Only some nodes react to live changes:

| Node | Reacts live to |
|---|---|
| `path_follower_node` | everything except `control_rate`, `auto_mode_on_service`, `auto_mode_off_service`, `inspection_services` |
| `obstacle_avoidance_node` | everything |
| `anomaly_detector_node` | `enabled`, `model`, `interval_sec`, `categories`, `alarm_categories`, `category_descriptions` |
| `person_detector_node` | `enabled`, `confidence`, `rate_hz` |
| all other nodes | nothing: `ros2 param set` is accepted, but the node only reads its parameters at start. Use the site file and restart |

The **Tuning** panel of RViz (`robot_rviz`) edits the navigation and avoidance parameters with
sliders; its **Save YAML…** button writes them to a file. RViz runs on a laptop with ROS 2
Humble on the robot's network: set `ROS_LOCALHOST_ONLY=0` in the env file, restart, and use
the robot's `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` on the laptop.

!!! danger "Set it back"
    With `ROS_LOCALHOST_ONLY=0` anyone on the robot's network can send motion commands to the
    robot. Set it back to `1` and restart when you are done.

## What the handheld changes

| On the handheld | Parameter | After a restart |
|---|---|---|
| Patrol speed slider | `path_follower_node` `target_linear_velocity`, from `patrol_speed_min` up to `max_linear_velocity` | the site file value |
| Inspect the floor | `anomaly_detector_node` `enabled` | kept |
| Pace: Continuous, Interval, Stop points | `interval_sec`: 0, the interval, -1 | kept |
| Model | `model` | kept |
| Finding types | `categories`, `alarm_categories`, `category_descriptions` | kept |
| Count people | `person_detector_node` `enabled` | kept |
| Speed 1, 2, 3 | 30, 60, 100 % of `teleop_udp_node` `max_vx`, `max_vy`, `max_wz` | a handheld setting |
| Inspection at a stop point | the dwell time stored in the route with each **Point** | in the route |

*Kept* values are stored in `~/.pashupati/inspection.json` and `~/.pashupati/people.json` and
**win over the site file**. To go back to the site file's inspection settings, stop the
service, delete `inspection.json`, and start it again.

## Tuning a new site, step by step

1. **The robot, once:** body size, body height, speed limits, stick channels (chapter 2).
2. **Braking:** measure it and test the protective field (chapter 3).
3. **AprilTags:** print, place, configure and record them (chapter 8).
4. **First route:** teach a short route and patrol it at 0.3 m/s (chapters 4 to 6).
5. **Obstacles:** a cardboard box on the route, then a person walking past (chapter 7).
6. **Full speed:** raise the patrol speed step by step, one full lap per step.
7. **AI and people:** pace, model, finding types, people counting (chapters 10 and 11).
8. **Save:** copy every value that works into the site file, restart, drive one more lap.

## Work safely

- Tune in a closed area first, never in a crowd.
- Keep the ZSBot remote in reach, and someone on the handheld's STOP.
- Change one thing at a time. Write down the old and the new value.
- Start low: speeds and accelerations below what you need, then raise them step by step.
- Repeat the protective field test (section 3.4) after every change to the body size, braking
  or speed limits.

# The Genisom L1W

## Body size

Pashupati treats the robot as a box around `base_link`, the middle of the body. Three nodes
use that box, and their values must agree:

| Node | Parameters | Used for |
|---|---|---|
| `control_mux_node` | `field.front`, `field.rear`, `field.half_width` | protective field (chapter 3) |
| `path_follower_node` | `footprint.front`, `footprint.rear`, `footprint.half_width` | path shift around obstacles (section 7.1) |
| `lidar_sector_node` | `self_filter_x_min`, `self_filter_x_max`, `self_filter_y_min`, `self_filter_y_max` | the robot's own legs, ignored by the LiDAR |

On the L1W the box is 0.80 × 0.56 m: front 0.40, rear 0.40, half width 0.28; self filter x from
-0.40 to 0.40 and y from -0.28 to 0.28. Measure again when the robot carries a payload or a
cover that sticks out.

## Body height

`base_link` is about 0.35 m above the floor when the L1W stands. Two parameters follow from it:

| Parameter | L1W | Rule |
|---|---|---|
| `lidar_sector_node` `obstacle_z_min` | -0.25 | minus the height of `base_link`, plus 0.10: anything less than 10 cm tall counts as floor |
| `person_detector_node` `floor_z` | -0.35 | minus the height of `base_link`: where people's feet are placed on the map |

To measure the height of `base_link`, stand the robot on a flat floor, measure the height of the
LiDAR above the floor, and subtract the LiDAR's height in the body:
`ros2 run tf2_ros tf2_echo base_link livox_frame` (the z value).

## Speed limits

| Parameter | L1W | Limits |
|---|---|---|
| `control_mux_node` `max_vx`, `max_vy`, `max_wz` | 1.0, 0.5, 1.5 | every command, AUTO and REMOTE: nothing may command more |
| `teleop_udp_node` `max_vx`, `max_vy`, `max_wz` | 1.0, 0.5, 1.5 | the handheld at speed level 3 (100 %) |
| `path_follower_node` `max_linear_velocity` | 0.6 | the patrol, and the top of the handheld's patrol speed slider |
| `l1w_driver_node` `stick_max_vx`, `stick_max_vy`, `stick_max_wz` | 1.0, 0.5, 1.5 | the command that means a full stick (next section) |

`max_vy` is 0 for a robot that cannot walk sideways. The L1W can, so keep `holonomic: true` in
`path_follower_node`.

## Stick channels and speed scaling

The L1W takes motion as joystick values, like its own remote. `l1w_driver_node` turns each
speed command into a stick position: a command of `stick_max_vx` is a full stick forward. Its
settings are in the robot package, `src/xtras/robots/genisom_l1w_ros2/l1w_driver/config/l1w_driver.yaml`.
Edit them there and restart the service; they are not live.

| Parameter | L1W | Meaning |
|---|---|---|
| `stick_max_vx`, `stick_max_vy`, `stick_max_wz` | 1.0, 0.5, 1.5 | speed that maps to full stick deflection |
| `stick_yaw_axis`, `stick_lateral_axis` | 2, 1 | stick channels for turning and walking sideways |
| `stick_invert_yaw`, `stick_invert_lateral` | false | true when the robot turns or walks sideways the wrong way |
| `cmd_timeout` | 0.5 | s; without a new command the stick returns to zero |
| `auto_control_right` | true | take motion control from the ZSBot remote when the driver connects |

**Check the directions once** (REMOTE, speed level 1): left stick to the left must walk the
robot to its left; right stick to the left must turn it to the left. If one is wrong, set its
`stick_invert_*` to true.

**Check the speed scale:** the speed the L1W reaches at full stick depends on its speed gear
(*Slow*, *Normal*, *Fast speed* in the robot actions). Patrol a straight route and compare the
patrol speed with the measured speed:
`ros2 topic echo /l1w/odom --field twist.twist.linear.x`. If the robot is slower or faster,
set `stick_max_vx` to its value × measured speed ÷ patrol speed, and scale `stick_max_vy` and
`stick_max_wz` the same way. Always patrol in the gear you checked.

!!! note
    The change is local to the robot package's clone. Commit it to `genisom_l1w_ros2` when it
    holds for every L1W, or note it in the robot's logbook so an update does not undo it.

# Protective field

## How it works

The control mux limits every motion command to a speed at which the robot can still stop
`field.margin` before the nearest obstacle in the path it is about to sweep. It looks in every
direction (360°, from the LiDAR's safety scan):

> *v* × *t* + *v*² ÷ 2*a* ≤ *free distance* − *margin*
>
> *v* speed · *t* `field.reaction_time` · *a* `field.max_decel` · *margin* `field.margin`

The free distance is measured from the robot's edge (chapter 2). With the defaults the robot may
move at most:

| Free distance from the edge | 0.3 m | 0.5 m | 0.75 m | 1.0 m | 1.5 m | 2.0 m |
|---|---|---|---|---|---|---|
| Top speed (m/s) | 0.30 | 0.52 | 0.71 | 0.87 | 1.13 | 1.35 |

The field works in AUTO. In REMOTE the operator watches the robot and nothing limits it, unless
`field_in_remote` is true. Without a fresh scan the robot does not drive itself at all.

## Parameters

All in `control_mux_node`, read at start:

| Parameter | Default | What it does |
|---|---|---|
| `field_in_auto` | true | limit AUTO (the patrol). Keep it on |
| `field_in_remote` | false | limit REMOTE too, e.g. for operators new to the robot |
| `field.max_decel` | 0.6 | m/s², braking the robot reliably achieves: **measure it** (section 3.3) |
| `field.reaction_time` | 0.25 | s, from the command to the robot actually braking |
| `field.margin` | 0.15 | m, gap left to the obstacle |
| `field.lateral_margin` | 0.10 | m, extra width of the swept path on each side |
| `field.min_speed` | 0.02 | m/s, slower commands are not limited (creeping, turning) |
| `safety_scan_timeout_sec` | 0.5 | s; an older scan stops AUTO and caps REMOTE |
| `stale_remote_speed` | 0.15 | m/s, the REMOTE cap while the scan is stale |

A lower `max_decel`, a longer `reaction_time` or a bigger `margin` make the robot slow down
earlier and stop further from obstacles. Too low, and it crawls in narrow aisles.

## Measure the braking

1. Teach a straight route of at least 8 m, with free space beyond it.
2. Set the patrol speed on the handheld to the fastest you will patrol at, for example 0.6.
3. Put a tape mark on the floor in the middle of the route.
4. Start the patrol. Press **STOP** when the front of the robot passes the mark.
5. Measure the distance *d* from the mark to the front of the stopped robot.
6. max_decel = speed² ÷ (2 × (*d* − speed × 0.25)).
7. Repeat three times. Use the smallest result.

Your own reaction is part of *d*, so the result errs on the safe side. At 0.6 m/s:

| Stopping distance *d* | 0.40 m | 0.50 m | 0.60 m | 0.80 m | 1.00 m |
|---|---|---|---|---|---|
| `field.max_decel` | 0.72 | 0.51 | 0.40 | 0.28 | 0.21 |

Measure on every new kind of floor: a polished floor brakes worse than a rough one.

## Test it

1. In the site file set `field_in_remote: true` and restart.
2. Put a cardboard box in an open area.
3. In REMOTE at speed level 1, drive straight at the box. The robot must slow down and stop
   about `field.margin` (15 cm) before it. Try level 2, then from the side and backwards.
4. Set `field_in_remote` back to false (or keep it on) and restart.

If the robot gets closer than the margin, lower `field.max_decel` or raise
`field.reaction_time`. Never test with a person in front of the robot.

## Control mux

| Parameter | Default | What it does |
|---|---|---|
| `remote_timeout_sec` | 0.3 | s; when the handheld goes silent the robot gets zero speed |
| `nav_timeout_sec` | 0.5 | s; the same for navigation in AUTO |
| `pause_nav_on_remote` | true | switching to REMOTE pauses the patrol |
| `estop_hold_hz` | 10 | zero commands per second while STOP is engaged |

# Speed and motion

## Cruise speed and curves

All in `path_follower_node`, live:

| Parameter | Default | What it does · when to change |
|---|---|---|
| `target_linear_velocity` | 0.4 | m/s, cruise speed; the handheld's patrol speed slider sets it. 0.3 in crowds |
| `max_linear_velocity` | 0.6 | m/s, highest cruise speed, also for the slider |
| `min_linear_velocity` | 0.1 | m/s, the speed profile never goes lower |
| `max_lateral_accel` | 0.3 | m/s², speed in curves: lower = slower in curves, steadier |
| `decel_limit` | 0.3 | m/s², braking planned into stop points and the route end |
| `approach_speed` | 0.15 | m/s, speed when arriving at a stop point or the route end |
| `max_angular_velocity` | 1.0 | rad/s, turn rate cap |

The speed in a curve is √(`max_lateral_accel` × radius): with 0.3, a curve of 1 m radius is
walked at 0.55 m/s and one of 0.5 m radius at 0.39 m/s.

## Acceleration and smoothness

| Parameter | Default | What it does · when to change |
|---|---|---|
| `linear_accel_limit` | 0.5 | m/s², how fast the speed may change. Jerky starts: lower it. A hard stop ignores it |
| `angular_accel_limit` | 1.5 | rad/s², the same for turning |
| `command_smoothing` | 0.4 | filter on turning and sideways commands: 1 = off, lower = smoother but slower to react. The body wags: lower it |
| `control_rate` | 20 | Hz, control loop rate (read at start) |

# Following the route

## Holonomic controller

`controller: holonomic` is the controller for legged robots. The L1W walks forward, sideways
or backwards as the route needs and keeps its heading along the route.

| Parameter | Default | What it does · when to change |
|---|---|---|
| `holonomic.lookahead_distance` | 0.6 | m, point on the route ahead the robot walks to. Longer: smoother, gentler return to the route, cuts corners more. Shorter: follows tightly, may weave |
| `holonomic.heading_lookahead` | 0.6 | m, the heading follows the route direction this far ahead. Longer: turns earlier into curves |
| `holonomic.kp_yaw` | 1.5 | heading correction strength. The body wags: lower it. Heading lags in curves: raise it |
| `holonomic.max_lateral_velocity` | 0.25 | m/s, sideways speed cap |
| `holonomic.max_reverse_velocity` | 0.15 | m/s, backwards speed when the route is behind the robot |
| `holonomic.min_alignment_scale` | 0.25 | speed factor while the heading is far off: low = turns first, then walks |
| `holonomic.goal_tolerance` | 0.3 | m, the end of an open route counts as reached |

## Leaving and joining the route

| Parameter | Default | What it does |
|---|---|---|
| `off_path_slow_distance` | 0.6 | m from the route: slow down to `min_linear_velocity` |
| `off_path_stop_distance` | 1.5 | m from the route: stop and report *Off the route*; the patrol needs a new **Start** |
| `join_max_distance` | 3.0 | m; started up to this far from the route, the robot first walks onto it. 0 = off |
| `join_timeout_sec` | 60 | s, give up walking onto the route and follow from where the robot is |

Sliding around an obstacle (section 7.1) does not count as being off the route.

## Route smoothing

`path_loader_node` smooths a recorded route between its stop points when it loads it. Stop
points never move.

| Parameter | Default | What it does · when to change |
|---|---|---|
| `smoothing_enabled` | true | smooth the route |
| `resample_spacing` | 0.1 | m between route points after smoothing |
| `smooth_weight_data` | 0.05 | pull toward the recording: higher keeps more of the wiggles you drove |
| `smooth_weight_smooth` | 0.4 | pull toward the neighbours: higher is smoother. Keep it below 0.5 |
| `max_smoothing_deviation` | 0.3 | m; the log warns when smoothing moved the route further |

If the smoothed route cuts a corner too close to a wall, lower `smooth_weight_smooth`, or record
that part again more carefully.

## Other controllers

`pure_pursuit`, `pid`, `mppi`, `lqr` and `stanley` only steer (forward and turn), for robots that
cannot walk sideways. With them, `speed_regulator_enabled` slows down in sharp turns. On the L1W
keep `holonomic`. Their gains are listed in the parameter reference.

# Stop points

## What happens at a stop

1. The robot slows down to `approach_speed` before the stop point (`decel_limit`).
2. Within `approach_radius` it switches to the final approach: it moves straight onto the
   recorded position and heading, walking sideways when needed.
3. It calls the inspection services and starts the dwell time stored in the route.
4. It moves on when the dwell time is over and the inspection answered, or after the dwell time
   plus `inspection_timeout_sec`.

A stop point the robot has already passed by more than `stop_point_skip_margin` route points
(0.5 m at the default spacing) is skipped, for example after a long detour.

## Final approach

| Parameter | Default | What it does · when to change |
|---|---|---|
| `approach_radius` | 0.5 | m from the stop point where the final approach starts |
| `approach.kp_xy` | 1.2 | position correction strength |
| `approach.kp_yaw` | 1.5 | heading correction strength |
| `approach.max_speed` | 0.2 | m/s |
| `approach.max_yaw_rate` | 0.6 | rad/s |
| `approach.min_speed` | 0.04 | m/s, lowest command. Must beat the joystick deadband: when the robot stops just short of the point, raise it to 0.05-0.08 |
| `approach.position_tolerance` | 0.05 | m, close enough |
| `approach.yaw_tolerance_deg` | 3 | deg, close enough |
| `approach_timeout_sec` | 20 | s; then the robot gives up and inspects where it is (the log says *approach timeout*) |

## Inspection at stops

| Parameter | Default | What it does |
|---|---|---|
| `inspection_services` | `[/anomaly_detector_node/check_now]` | services called at every stop point (read at start) |
| `inspection_timeout_sec` | 20 | s the robot waits for them beyond the dwell time |

The dwell time comes from the route: the handheld's *Inspection at a stop point* setting is
stored with each **Point** you mark (10 s unless changed).

# Obstacles

## Path shift

With `avoidance_mode: path_shift` (the default with the holonomic controller) the robot passes an
obstacle by sliding sideways along a line parallel to the route, heading kept. When there is no
gap wide enough on either side, it slows down and waits in front of the obstacle. After
`blocked_timeout_sec` it reports *Blocked* and keeps waiting until the way is free.

| Parameter | Default | What it does · when to change |
|---|---|---|
| `path_shift.max_offset` | 0.8 | m, furthest the robot leaves the route. Narrow aisles: lower it |
| `path_shift.look_ahead` | 2.0 | m of route ahead of the front edge that must be free |
| `path_shift.clearance` | 0.10 | m kept between the robot's side and an obstacle. It does not pass gaps that look wide enough: lower it a little |
| `path_shift.offset_rate` | 0.3 | m/s, how fast it slides sideways |
| `path_shift.switch_cost` | 0.5 | higher: keeps its side longer instead of switching |
| `path_shift.switch_margin` | 0.05 | m, extra room a new side needs (no flicker) |
| `path_shift.return_delay_sec` | 0.5 | s, the way back to the route must stay free this long |
| `path_shift.decel` | 0.5 | m/s², braking toward an obstacle it cannot pass |
| `path_shift.stop_margin` | 0.40 | m, stops this far before an obstacle (from the front edge) |
| `blocked_timeout_sec` | 30 | s without progress before *Blocked* is reported |

## Hard stop

`obstacle_avoidance_node` stops the robot at once, in every mode, when anything is closer than
`emergency_distance` to `base_link` inside the front cone of `emergency_cone_deg`. The robot
continues when the way stayed free for `estop_release_sec` (1 s, `path_follower_node`).

| Parameter | Default | What it does |
|---|---|---|
| `emergency_distance` | 0.75 | m from `base_link`: about 0.35 m in front of the L1W's nose |
| `emergency_cone_deg` | 60 | deg, width of the front cone |
| `safe_distance` | 1.0 | m; closer obstacles shape the command (reactive modes only) |

Keep `footprint.front` + `path_shift.stop_margin` (0.80 m) above `emergency_distance`
(0.75 m): the robot then normally stops smoothly in front of an obstacle, and the hard stop only
catches what appears suddenly.

## What the LiDAR sees

`lidar_sector_node` turns the LiDAR points into the obstacle scans. Read at start.

| Parameter | Default | What it does · when to change |
|---|---|---|
| `input_type` | livox | `livox` reads the Livox driver directly; keep it on the L1W |
| `obstacle_z_min` | -0.25 | m in `base_link`: lower points are floor. Floor or ramps seen as obstacles: raise it. Low objects missed: lower it |
| `obstacle_z_max` | 1.20 | m in `base_link`: higher points are ignored (signs, ceilings) |
| `self_filter_*` | ±0.40, ±0.28 | m, the robot's own body. *Slowed* with nothing around: the legs are seen, widen it a little |
| `range_min`, `range_max` | 0.2, 8.0 | m, LiDAR range used |
| `safety_num_sectors` | 72 | sectors of the 360° safety scan (5° each) |
| `safety_range_max` | 4.0 | m; further obstacles never limit the speed |
| `min_raw_points` | 100 | fewer points per scan means the LiDAR is covered or failed: everything counts as blocked |

!!! warning
    The LiDAR does not see drops (stairs, platform edges), glass, mirrors or thin poles well.
    Keep routes away from them; no parameter fixes that.

## Reactive avoidance

`avoidance_mode: reactive` (and every non-holonomic controller) uses the steering of
`obstacle_avoidance_node` instead of the path shift. `algorithm` picks it: `vfh` (default),
`braitenberg`, `potential_field` or `follow_gap`. Each has a `steering_gain` (how hard obstacles
turn the robot), a `velocity_gain` (how much they slow it) and a `smoothing` filter (0-1, higher
is smoother and slower). The L1W uses the path shift; the gains are in the parameter reference.

# Localization

## AprilTags

The robot finds itself on the map from AprilTags: printed squares whose positions on the map are
saved in `tag_config.json` (`TAGS` in the env file). The `apriltag` node in
`perception_params.yaml` must know every tag:

| Parameter | Default | What it does |
|---|---|---|
| `family` | 36h11 | tag family; print tags of this family |
| `tag.ids` | [2, 4] | IDs of the tags in use |
| `tag.frames` | [base_map, charging_dock] | a name for each ID, in the same order |
| `tag.sizes` | [0.15, 0.15] | m, edge of each tag's black square, in the same order |
| `size` | 0.16 | m, edge of any other tag |
| `detector.decimate` | 2.0 | image shrink before detection: 1.0 sees tags further away, uses more CPU |
| `max_hamming` | 0 | bit errors allowed; keep 0 (no false tags) |
| `apriltag_throttle` `rate_hz` | 5 | camera frames per second given to the detector |

The three `tag.*` lists must have the same length, and `localization_node` `tag_frames` must list
the same names.

## Placing and recording tags

- Measure the printed size across the black square, not the white margin, and enter it in
  `tag.sizes`. A wrong size puts the robot in the wrong place.
- Mount tags flat and rigid on walls at about the camera's height, where the robot stands still
  facing them: at the start of the route, and at places where it should correct drift.
- Bigger tags are seen from further away. Avoid glare and direct sunlight on the tag.
- After mounting, record them from the handheld (**Tags**, see the User Manual). The positions are
  averaged over `tag_record_duration_sec` and saved to `tag_config.json`; the old file is kept as
  a `.bak`. Moving a tag later means recording it again.

## Localization node

| Parameter | Default | What it does · when to change |
|---|---|---|
| `auto_localize_on_start` | true | try to localize from a tag every second after start, until it works |
| `assume_start_origin` | false | true: no tag needed, the start pose is the map origin. Then the robot must always start at exactly the same spot and heading |
| `auto_correct_period_sec` | 0 | s; > 0 re-corrects drift from any visible tag this often. 30 for long patrols past tags |
| `max_correction_m` | 0.5 | m; larger automatic corrections are rejected |
| `max_correction_deg` | 15 | deg, the same for heading |
| `tag_max_age_sec` | 0.5 | s; older tag sightings are ignored |
| `map_odom_hz` | 20 | how often the map correction is republished |

## Odometry checks

FAST-LIO, the LiDAR odometry, tracks the robot between tags. It needs a still, level start:
`odometry_gate` starts it only when the LiDAR has been still for `still_sec` (2 s) and tilts less
than `max_tilt_deg` (8°). After `timeout_sec` (60 s) it starts anyway and the health status
reports an error. Give the robot a still moment after standing up.

`odometry_monitor_node` watches the running odometry and reports to `/health/status`:

| Parameter | Default | Reports when |
|---|---|---|
| `stale_sec` | 0.5 | no odometry for this long |
| `max_speed_mps` | 3.0 | the pose jumps faster than this |
| `max_z_drift_m` | 0.5 | the height drifts (tilted start). 0 = off |
| `max_speed_diff_mps` | 0.35 | LiDAR and leg speeds differ this much ... |
| `speed_diff_sec` | 1.5 | ... for this long (slipping, featureless corridor) |
| `restart_origin_radius_m`, `restart_min_offset_m` | 0.05, 0.3 | FAST-LIO restarted: the pose jumped back to its origin |

These rarely need tuning. A *speed mismatch* in one long, plain corridor is a hint to add a tag
there and use `auto_correct_period_sec`.

# Patrol loop and fail-safes

All in `path_follower_node`, live:

| Parameter | Default | What it does |
|---|---|---|
| `loop_route` | true | restart a closed route when it ends |
| `loop_close_distance` | 1.0 | m; a route whose end is this close to its start is closed |
| `low_battery_percentage` | 0.2 | 0-1; below it the patrol stops at the end of the lap |
| `require_localization` | true | refuse **Start** until the robot is localized |
| `estop_release_sec` | 1.0 | s the way must stay clear after a hard stop |
| `avoidance_timeout_sec` | 0.5 | s; stop when the obstacle scan goes silent |
| `tf_timeout_sec` | 0.5 | s; stop when the robot pose stops updating (odometry died) |
| `auto_mode_on_service`, `auto_mode_off_service` | "" | services called when a patrol starts and stops (read at start) |

On the L1W `auto_mode_*_service` stay empty: the driver takes control when it connects
(`auto_control_right`), and **Take control** on the handheld takes it back after the ZSBot
remote was used.

# AI inspection

## Pace and queue

`anomaly_detector_node` sends camera images to an AI vision model (Ollama, on the robot) and
gets back one finding type or *none*.

| Parameter | Default | What it does · when to change |
|---|---|---|
| `enabled` | true | live; off also unloads the model from memory |
| `interval_sec` | 0 | live: 0 continuous, > 0 one image every N s, < 0 only at stop points |
| `queue_max` | 50 | interval pace: images waiting for the model; when full, the oldest is dropped |
| `queue_min_change` | 12 | grey levels the picture must change to be queued again: standing still adds nothing. 0 = every image |
| `queue_dir` | `~/.pashupati/inspection_queue` | waiting images survive a restart; "" = memory only |
| `cooldown_sec` | 60 | s; continuous and interval pace do not report the same type again within this time |
| `max_image_age_sec` | 2.0 | s; older camera images are not inspected |
| `max_image_width` | 640 | px; images are shrunk first. Smaller is faster but misses small things |

One inspection takes about 1-2 s on the Jetson. An interval shorter than that fills the queue;
that is fine while the robot stands at stops, where the queue drains.

## Model and memory

| Parameter | Default | What it does |
|---|---|---|
| `model` | qwen3-vl:2b-instruct | live; the model in use |
| `models` | three models | offered on the handheld (plus any other vision model in Ollama) |
| `keep_alive` | "-1" | keep the model loaded: "-1" forever, or e.g. "30m" (quoted) |
| `num_ctx` | 2048 | tokens of context: the image and the type list must fit. Many long descriptions: 4096 (more memory) |
| `timeout_sec` | 30 | s, longest wait for one answer |
| `use_schema` | true | structured answers; false only for models that cannot do them |
| `describe` | false | true: the model also writes one sentence about the floor (slower) |
| `debug` | false | true: log every inspection with the raw model answer |

A model must be pulled before it can be used (`ollama pull <model>`, or
`script/setup.sh install ollama` with `VLM_EXTRA_MODELS`). `setup.sh install ollama` also turns off
Ollama's prompt cache, which otherwise grows to several GB of RAM. Check with `free -h`: with
the model loaded the robot normally uses about 7 GB of its 15 GB.

## Finding types

| Parameter | Default | What it does |
|---|---|---|
| `categories` | [trash, spill, stain] | live; the types the model chooses from (plus *none*), up to 12, a-z 0-9 _ |
| `alarm_categories` | [trash, spill] | live; only these are reported to NETRA, the rest is shown on the handheld |
| `category_descriptions` | [""] | live; own text per type, `"name: what to look for"` |

`trash`, `spill`, `stain`, `floor_damage` and `fallen_person` have built-in descriptions; any other
type needs one. The handheld edits all three lists (**Inspect → Finding types**); its changes
win over the site file (section 1.4). Good descriptions:

- are in English, short and concrete: *a bag, backpack or suitcase standing alone on the floor*;
- name what is visible in the image, not what it means;
- say what is **not** a finding when the model confuses things: *not shadows, reflections or
  floor patterns*.

Too many false *stain* findings on a shiny floor is why `stain` is not reported by default.

## Reporting

`fleet_uplink_node` sends findings to NETRA:

| Parameter | Default | What it does |
|---|---|---|
| `anomaly_min_confidence` | 0.5 | 0-1; less confident findings are not sent |
| `spool_dir` | `~/.pashupati/spool` | findings wait here while NETRA cannot be reached |

# People counting

`person_detector_node` finds people in the camera image with YOLO11n. Started only with
`PEOPLE=true` in the env file; the handheld switches it on and off.

| Parameter | Default | What it does · when to change |
|---|---|---|
| `enabled` | true | live; on or off |
| `confidence` | 0.4 | live, 0-1. Boxes on things that are not people: raise it. People missed: lower it |
| `rate_hz` | 0 | live; detections per second. 0 = 5 on the GPU, 2 on the CPU |
| `threads` | 2 | CPU threads for one detection (CPU only) |
| `floor_z` | -0.35 | m, floor height in `base_link` (section 2.2) |
| `max_range` | 12 | m; people further away are not placed on the map |
| `backend` | auto | `onnxruntime`, `opencv` or `ultralytics`; auto picks the first that works |
| `model_path` | `models/yolo11n-416.onnx` | installed by `script/setup.sh install people_model` |

NETRA's crowd map uses `fleet_uplink_node` `crowd_cell_size` (1 m grid) and `crowd_window_sec`
(one sample per minute, the peak per cell).

# Handheld, video, NETRA and health

## Handheld link

`teleop_udp_node`, read at start:

| Parameter | Default | What it does |
|---|---|---|
| `token` | "" | shared secret; the same in the handheld's connection settings. **Always set it** |
| `robot_name` | "" | name shown on the handheld's start screen (empty: the computer's name) |
| `port` | 9870 | UDP port of the link and of robot discovery |
| `client_timeout_sec` | 2.0 | s; a silent handheld is dropped |
| `robot_commands` | see chapter 15 | the robot actions on the handheld, in this order |
| `patrol_speed_min` | 0.1 | m/s, slowest patrol speed the slider offers |
| `lidar_hz`, `lidar_voxel`, `lidar_max_points` | 2, 0.1, 3000 | LiDAR points on the handheld's map: fewer is lighter on Wi-Fi |

## Video

The handheld gets H.264 video over WebRTC from `video_stream_node`, started while it watches:

| Parameter | Default | What it does |
|---|---|---|
| `bitrate_kbps` | 1500 | weak Wi-Fi, stuttering video: 800 |
| `fps` | 15 | frames per second: 10 saves Wi-Fi and CPU |
| `max_width` | 1280 | px; wider images are shrunk |
| `keyframe_sec` | 1.0 | s; a new viewer starts at a key frame, keep it short |
| `encoder` | auto | the Jetson's hardware encoder when it works |

Without WebRTC (`WEBRTC=false` or MediaMTX missing) the handheld gets JPEG frames instead:
`teleop_udp_node` `camera_fps` (8), `camera_jpeg_quality` (60), `camera_max_width` (640). The
camera itself runs at 640 × 480, 15 fps (`l1w_bringup/config/realsense_params.yaml`).

## NETRA uplink

`fleet_uplink_node`:

| Parameter | Default | What it does |
|---|---|---|
| `server_url` | "" | NETRA address; set by `CC_URL` in the env file |
| `token` | "" | the robot's token from NETRA; set by `CC_TOKEN` |
| `robot_id` | robot-01 | the robot's name in NETRA; `ROBOT_ID` in the env file wins |
| `telemetry_hz` | 1.0 | position and status messages per second |
| `upload_period_sec` | 5.0 | s between upload attempts of waiting findings and routes |
| `verify_tls` | true | check NETRA's certificate; keep it on |

## Health monitor

`health_monitor_node` publishes `/health/status`, shown on the handheld and in NETRA:

| Parameter | Default | Issue when |
|---|---|---|
| `topics` | LiDAR, odometry, scans, camera, robot odometry | a topic runs below 80 % of its listed rate (`"topic:Hz"`) |
| `max_cpu_temp_c` | 85 | the CPU is hotter |
| `max_memory_percent` | 92 | more memory is in use |
| `min_disk_free_gb` | 5 | less disk is free |
| `startup_grace_sec` | 30 | s after start without missing-node issues |

# The env file

`/etc/pashupati/pashupati.env` says what runs. Restart the service after a change. Quote values
with spaces, and never put a comment after a value.

| Key | Genisom L1W | Meaning |
|---|---|---|
| `ROBOT_ID` | e.g. `l1w-01` | the robot's name in NETRA |
| `ROBOT_BRINGUP` | `l1w_bringup` | robot package started with the stack |
| `ROBOT_NS` | `l1w` | namespace of the driver's topics and services |
| `ROBOT_PRESTART` | `"/usr/local/bin/robot-launch stop driver-livox forward"` | stops the vendor's own LiDAR driver and motion stack first |
| `ROBOT_SETUP` | `…/genisom_l1w_ros2/script/setup.sh` | the robot package's installer (ZSBot SDK) |
| `ROBOT_IP` | `192.168.234.1` | motion computer, for the network check |
| `MODE` | `patrol` | `teach`: no navigation, the LiDAR map is saved |
| `ODOMETRY` | `fast_lio` | or `leg_kilo` |
| `ROUTE` | empty | route loaded at start; empty: the last one loaded |
| `TAGS` | `…/map/tag_config.json` | AprilTag positions |
| `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION` | `0`, `rmw_fastrtps_cpp` | Pashupati's own DDS; the vendor stack uses another |
| `ROS_LOCALHOST_ONLY` | `1` | `0` only for RViz from a laptop (section 1.3) |
| `PEOPLE`, `INSPECTION` | `true` | start people counting and the AI inspection at all |
| `SITE_CONFIG` | `/etc/pashupati/site.yaml` | the site file |
| `WEBRTC`, `MEDIAMTX` | `true`, empty | WebRTC video for the handheld |
| `CC_URL`, `CC_TOKEN` | NETRA's address, the robot's token | NETRA uplink; empty: findings are kept on the robot |
| `TIME_SYNC_WAIT` | `60` | s to wait at boot for the network clock |
| `OPENBLAS_NUM_THREADS`, `OMP_NUM_THREADS` | `1` | keep at 1: more threads waste CPU |

`script/setup.sh check` reports a missing key, or a line that breaks the rules above.

# Symptoms and fixes

| Symptom | Look at | Try |
|---|---|---|
| Weaves left and right on straight parts | `holonomic.lookahead_distance`, `holonomic.kp_yaw`, `command_smoothing` | lookahead 0.8, kp_yaw 1.0, smoothing 0.3 |
| Swings wide or cuts corners | `holonomic.lookahead_distance`, `max_lateral_accel` | lookahead 0.4-0.5, lateral accel 0.2 |
| Too slow in gentle curves | `max_lateral_accel` | 0.4 |
| Jerky starts and stops | `linear_accel_limit`, `decel_limit` | 0.3, 0.2 |
| Walks slower or faster than the patrol speed | `stick_max_*` (section 2.4), speed gear | rescale; patrol in the checked gear |
| Turns or walks sideways the wrong way | `stick_invert_yaw`, `stick_invert_lateral` | true |
| Stops just short of a stop point and waits 20 s | `approach.min_speed`, `approach.position_tolerance` | 0.06, 0.08 |
| Leaves stop points too late | dwell time of the route, `inspection_timeout_sec`, AI pace | shorter dwell, faster model |
| *Slowed* or *Blocked* with nothing around | `self_filter_*`, `obstacle_z_min`, floor slopes | widen the filter 2-3 cm; raise z_min 0.03 |
| Ignores low objects (bags, boxes) | `obstacle_z_min` | lower 0.03 at a time, check the floor is not seen |
| Stops far before obstacles | `path_shift.stop_margin`, `field.margin`, `field.max_decel` | small steps; repeat the test of section 3.4 |
| Gets too close to obstacles | `field.max_decel`, `field.reaction_time` | measure the braking again (section 3.3) |
| Does not pass through gaps that look wide enough | `path_shift.clearance`, `path_shift.max_offset`, `footprint.half_width` | clearance 0.05, larger offset |
| Switches sides around an obstacle | `path_shift.switch_cost`, `path_shift.switch_margin` | 1.0, 0.10 |
| *Blocked* too soon or too late | `blocked_timeout_sec` | to taste |
| *Off the route* after a push | `off_path_stop_distance` | 2.0, if the area allows |
| **Start** refused: robot too far from the route | `join_max_distance` | drive closer, or raise it |
| *No loc*: tag not found | `tag.sizes`, `tag.ids`, light, `detector.decimate` | measure the tag, decimate 1.0 |
| Position drifts over long patrols | `auto_correct_period_sec` | 30, with tags along the route |
| Same trash reported again and again | `cooldown_sec` | 120-300 |
| False findings | type description, `alarm_categories`, `anomaly_min_confidence` | describe what is not a finding; 0.6-0.7 |
| Inspection queue always full | `interval_sec`, `model`, `max_image_width` | longer interval, smaller model |
| RAM near the limit | Ollama prompt cache, `keep_alive`, extra models | `setup.sh install ollama`; inspection off unloads the model |
| People count jumps | `confidence`, `rate_hz` | 0.5, 2 |
| Video stutters | `bitrate_kbps`, `fps` | 800, 10 |
| Handheld map lags | `lidar_hz`, `lidar_voxel`, `lidar_max_points` | 1, 0.2, 1500 |
| Findings missing in NETRA | `CC_URL`, `CC_TOKEN`, `alarm_categories`, `anomaly_min_confidence` | check the log of `fleet_uplink_node` |

# Site file for the Genisom L1W

A complete starting point for `/etc/pashupati/site.yaml`, in two parts: the robot, and the
site. Measure and change the values marked MEASURE. Both parts go in the same file.

## The robot part

```yaml
control_mux_node:
  ros__parameters:
    field.front: 0.40
    field.rear: 0.40
    field.half_width: 0.28
    field.max_decel: 0.6          # MEASURE on this floor (3.3)
    field.reaction_time: 0.25
    max_vx: 1.0
    max_vy: 0.5
    max_wz: 1.5

path_follower_node:
  ros__parameters:
    holonomic: true
    target_linear_velocity: 0.35  # cruise speed for this site
    low_battery_percentage: 0.25

lidar_sector_node:
  ros__parameters:
    obstacle_z_min: -0.25         # MEASURE: -(base_link height) + 0.10
    self_filter_x_min: -0.40
    self_filter_x_max: 0.40
    self_filter_y_min: -0.28
    self_filter_y_max: 0.28

person_detector_node:
  ros__parameters:
    floor_z: -0.35                # MEASURE: -(base_link height)

teleop_udp_node:
  ros__parameters:
    token: "change-me"            # the same in the handheld
    max_vx: 1.0
    max_vy: 0.5
    max_wz: 1.5
    robot_commands: [stand_up, sit_down, balance_stand_mode,
                     move_mode, lock_mode, slow_speed,
                     normal_speed, fast_speed,
                     sdk_control_right, emergency_stop]
```

## The site part

```yaml
obstacle_avoidance_node:
  ros__parameters:
    emergency_distance: 0.75

apriltag:                         # the tags on this site's walls
  ros__parameters:
    tag:
      ids: [2, 4]
      frames: [base_map, charging_dock]
      sizes: [0.15, 0.15]         # MEASURE the printed square

localization_node:
  ros__parameters:
    tag_frames: [base_map, charging_dock]   # as tag.frames
    auto_correct_period_sec: 0.0

fleet_uplink_node:
  ros__parameters:
    robot_id: l1w-01              # ROBOT_ID in the env file wins
```

Each node may appear only once in the file: add more parameters under its existing heading.
After a restart, `ros2 param dump /control_mux_node` (and so on) shows the values in use.

# Parameter reference

Every parameter of every Pashupati node with its default value, read from the parameter files of
this version. *Meaning* is the comment in the file. Live parameters are listed in section 1.3;
change the others in the site file and restart.

{{ parameter_reference }}

# Contributing

## Branches

| Branch                                           | Purpose                                                                               |
| ------------------------------------------------ | ------------------------------------------------------------------------------------- |
| `dev`                                            | integration branch, always builds and passes CI; the robot runs a tagged commit of it |
| `fix/<topic>`                                    | bug fixes, small and reviewable, merged first                                         |
| `feat/<topic>`                                   | one feature per branch, with tests and docs                                           |
| `refactor/<topic>`, `ci/<topic>`, `docs/<topic>` | no behaviour change (or a documented breaking change)                                 |

Open a pull request into `dev`; CI (`.github/workflows/ci.yml`) must be green. Stacked branches
(one built on another) say so in the PR description and are merged in that order. Releases are
tags `vX.Y.Z` on `dev` with an entry in [CHANGELOG.md](CHANGELOG.md).

Commits follow [Conventional Commits](https://www.conventionalcommits.org):
`fix(navigation): ...`, `feat(bridge): ...`, `refactor(bringup)!: ...` plus a `BREAKING CHANGE:`
footer when users have to change something.

## Build and test

```bash
scripts/dev.sh up && scripts/dev.sh build && scripts/dev.sh test  # dev container (macOS/Linux)
colcon build && colcon test --executor sequential && colcon test-result --verbose   # on the robot
pip install pre-commit && pre-commit install                       # same lint as CI, on commit
```

The lint is the Dharma hub's: ruff (`ruff.toml`), prettier for YAML and Markdown, clang-format
(`.pre-commit-config.yaml`). Every package runs `test/test_style.py` (dharma_utils'
`check_package`) and its own tests in `colcon test`. Tests are split into `test/unit` (pure, plain
`pytest` on a laptop) and `test/ros` (in-process nodes with fake data, skipped without `rclpy`);
`src/pashupati/test/e2e` runs the whole robot on the simulator.

## Package layout

Like every Dharma package: `core/` is pure Python (no rclpy, no I/O at import), `utils/` holds
the ROS-side helpers, and the nodes are thin `DharmaNode`s that read a typed config with
`declare_config()` and run through `dharma_utils.ros.run()`. Modules stay under 300 lines (tests
under 400). Status goes out as the typed `dharma_interfaces` messages, never parsed from strings.

## Rules that come from real crashes

- **Never redefine an rclpy `Node` member** (`handle`, `context`, `executor`, `_clock`, ...) in a
  node class (`self.clients` was one). `test/test_style.py` in every package enforces it.
- **One logging call site per severity.** rclpy raises `Logger severity cannot be changed between
calls` when the same line logs once as `info` and once as `warn`; write two calls instead of
  `log = logger.warn if bad else logger.info`.
- **Stop cleanly:** `main()` is `dharma_utils.ros.run(NodeClass)`, which handles Ctrl-C and an
  external shutdown; a node that starts a thread stops it in a shutdown hook
  (`add_shutdown_hook`) and never publishes after it was destroyed.
- **TF through `self.tf`** (the DharmaNode's cache, on the node's own executor), never a
  `TransformListener` with `spin_thread=True`, which adds the whole node to a second executor.
- **One BLAS thread per process.** pip numpy's OpenBLAS threads busy-wait between the small matrix
  operations nodes run at 10-50 Hz and burn whole cores; `run_robot.sh` and `scripts/env.sh` set
  `OPENBLAS_NUM_THREADS=1`.
- **No per-element Python objects at sensor rate.** rclpy builds one object per element of a
  message array (a Livox CustomMsg has ~20k points): read big messages as numpy straight from the
  buffer (`PointCloud2.data`, or `raw=True` and the CDR bytes, see `livox_cdr.py` in dharma_perception).
- **Subscribe to heavy topics only while someone uses them** (the teleop camera and cloud).
- **Validate everything from the network** (teleop UDP, NETRA commands, spool files): clamp
  numbers, reject non-finite values, keep file paths inside their directory, never raise in a
  callback.
- **Parameters** are typed config fields (`param(default, min, max, unit, doc)`) with the package
  `config/*.yaml` as the defaults; the robot's tuned values live in `src/pashupati/config`, and
  one robot is tuned in `/etc/pashupati/site.yaml`, never by editing the repository on the robot.
- **The env file** (`/etc/pashupati/pashupati.env`) is read by both systemd and bash: no comment
  after a value (systemd makes it part of the value) and quotes around a value with spaces (bash
  runs the second word as a command). `scripts/setup.sh check` flags both.
- **Signal only `ros2 launch`.** Stop the stack with one SIGINT to the launch process (systemd does
  this with `KillMode=mixed`), not to its process group: launch forwards it to every node, and a
  second SIGINT interrupts the nodes in the middle of their shutdown.

## Safety-relevant changes

Anything that changes how the robot moves (speeds, limits, e-stop, avoidance, the control mux, the
teleop gateway) needs a test that drives the logic, a note in the PR on how it was tried on the robot
(slow speed, open area, someone at the e-stop), and a CHANGELOG entry. Try it in the simulator
first (`ros2 launch pashupati sim.launch.py rviz:=true`); its end-to-end test (a taught route
patrolled with inspections, the handheld taking over through the control mux) runs in CI and must
stay green.

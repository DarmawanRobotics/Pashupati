# Contributing

## Branches

| Branch | Purpose |
|---|---|
| `dev` | integration branch, always builds and passes CI; the robot runs a tagged commit of it |
| `fix/<topic>` | bug fixes, small and reviewable, merged first |
| `feat/<topic>` | one feature per branch, with tests and docs |
| `refactor/<topic>`, `ci/<topic>`, `docs/<topic>` | no behaviour change (or a documented breaking change) |

Open a pull request into `dev`; CI (`.github/workflows/ci.yml`) must be green. Stacked branches
(one built on another) say so in the PR description and are merged in that order. Releases are
tags `vX.Y.Z` on `dev` with an entry in [CHANGELOG.md](CHANGELOG.md).

Commits follow [Conventional Commits](https://www.conventionalcommits.org):
`fix(navigation): ...`, `feat(bridge): ...`, `refactor(bringup)!: ...` plus a `BREAKING CHANGE:`
footer when users have to change something.

## Build and test

```bash
script/dev.sh up && script/dev.sh build && script/dev.sh test     # dev container (macOS/Linux)
colcon build && colcon test --executor sequential && colcon test-result --verbose   # on the robot
pip install pre-commit && pre-commit install                      # same lint as CI, on commit
```

Every Python package runs `ament_flake8`, `ament_pep257` and its own tests in `colcon test`. Tests
that need ROS (an in-process node fed with fake data) skip themselves when `rclpy` is missing, so
pure-logic tests also run with plain `pytest` on a laptop.

## Rules that come from real crashes

- **Never redefine an rclpy `Node` member** (`handle`, `context`, `executor`, `_clock`, ...) in a
  node class. `test/test_node_members.py` in every package enforces it.
- **One logging call site per severity.** rclpy raises `Logger severity cannot be changed between
  calls` when the same line logs once as `info` and once as `warn`; write two calls instead of
  `log = logger.warn if bad else logger.info`.
- **Stop cleanly:** `main()` catches `KeyboardInterrupt` and `ExternalShutdownException` and wraps
  `rclpy.shutdown()` in `try/except`; a node with a spin-thread `TransformListener` stops that thread
  in `destroy_node()`. A Ctrl-C can land while the executor builds its wait set, so any other
  exception from `spin()` is a normal stop when `rclpy.ok()` is already false; re-raise it only
  while `rclpy.ok()` is true. Copy the `main()` of an existing node.
- **Validate everything from the network** (teleop UDP, NETRA commands, spool files): clamp
  numbers, reject non-finite values, keep file paths inside their directory, never raise in a
  callback.
- **Parameters** live in the package `config/*.yaml` with a comment, are listed in the package
  README, and are tuned per robot in `/etc/pashupati/site.yaml`, never by editing the repository on
  the robot.
- **systemd env files** (`/etc/pashupati/pashupati.env`) take no comments after a value: systemd
  makes the comment part of the value.

## Safety-relevant changes

Anything that changes how the robot moves (speeds, limits, e-stop, avoidance, the control mux, the
teleop gateway) needs a test that drives the logic, a note in the PR on how it was tried on the robot
(slow speed, open area, someone at the e-stop), and a CHANGELOG entry.

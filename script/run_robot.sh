#!/usr/bin/env bash
# Run the robot stack in the foreground (systemd ExecStart). Settings come from the environment,
# normally /etc/pashupati/pashupati.env; robot.launch.py reads every argument from it.
set -eo pipefail

WS="${WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
set -u

# Robot-specific preparation, e.g. stopping a vendor autostart that would fight for the robot.
if [[ -n "${ROBOT_PRESTART:-}" ]]; then
    bash -c "$ROBOT_PRESTART" || echo "ROBOT_PRESTART failed, starting anyway" >&2
fi

# numpy's OpenBLAS threads busy-wait between the small matrix ops the nodes run at
# 10-50 Hz and burn whole cores; one BLAS thread per process is plenty.
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}" OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}"

export ROS_LOG_DIR="${ROS_LOG_DIR:-$HOME/.ros/log}"
# Keep ROS logs from filling the disk: delete runs older than a week.
find "$ROS_LOG_DIR" -mindepth 1 -maxdepth 1 -mtime +7 -exec rm -rf {} + 2>/dev/null || true

# Version shown in the first log line and in the journal
PASHUPATI_GIT="$(git -C "$WS" describe --always --dirty --tags 2>/dev/null || echo unknown)"
export PASHUPATI_GIT

exec ros2 launch pashupati robot.launch.py

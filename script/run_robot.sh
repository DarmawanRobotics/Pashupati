#!/usr/bin/env bash
# Run the robot stack in the foreground (systemd ExecStart). Settings come from the environment,
# normally /etc/pashupati/pashupati.env: ROBOT_ID, ROBOT_NS, ROBOT_BRINGUP, ROBOT_PRESTART, MODE,
# ODOMETRY, ROUTE, TAGS, CC_URL, CC_TOKEN, SITE_CONFIG, ROS_DOMAIN_ID, PEOPLE, INSPECTION.
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

export ROS_LOG_DIR="${ROS_LOG_DIR:-$HOME/.ros/log}"
# Keep ROS logs from filling the disk: delete runs older than a week.
find "$ROS_LOG_DIR" -mindepth 1 -maxdepth 1 -mtime +7 -exec rm -rf {} + 2>/dev/null || true

# Version shown in the first log line and in the journal
PASHUPATI_GIT="$(git -C "$WS" describe --always --dirty --tags 2>/dev/null || echo unknown)"
export PASHUPATI_GIT

exec ros2 launch pashupati robot.launch.py \
    robot_id:="${ROBOT_ID:-}" \
    mode:="${MODE:-patrol}" \
    odometry:="${ODOMETRY:-fast_lio}" \
    route:="${ROUTE:-}" \
    tags:="${TAGS:-}" \
    robot_ns:="${ROBOT_NS:-robot}" \
    robot_bringup:="${ROBOT_BRINGUP:-}" \
    cc_url:="${CC_URL:-}" \
    cc_token:="${CC_TOKEN:-}" \
    people:="${PEOPLE:-true}" \
    inspection:="${INSPECTION:-true}" \
    site_config:="${SITE_CONFIG:-/etc/pashupati/site.yaml}"

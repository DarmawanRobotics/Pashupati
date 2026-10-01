#!/usr/bin/env bash
# Run the robot stack in the foreground (systemd ExecStart). Settings come from the environment,
# normally /etc/pashupati/pashupati.env: MODE, ROUTE, TAGS, ROBOT_NS, CC_URL, CC_TOKEN, ROS_DOMAIN_ID,
# PEOPLE, INSPECTION.
set -eo pipefail

WS="${WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
set -u

# The vendor autostart fights for the robot; stop it first.
if command -v robot-launch >/dev/null; then
    robot-launch stop forward || true
fi

export ROS_LOG_DIR="${ROS_LOG_DIR:-$HOME/.ros/log}"
# Keep ROS logs from filling the disk: delete runs older than a week.
find "$ROS_LOG_DIR" -mindepth 1 -maxdepth 1 -mtime +7 -exec rm -rf {} + 2>/dev/null || true

exec ros2 launch pashupati_bringup robot.launch.py \
    mode:="${MODE:-patrol}" \
    route:="${ROUTE:-}" \
    tags:="${TAGS:-}" \
    robot_ns:="${ROBOT_NS:-l1w}" \
    cc_url:="${CC_URL:-}" \
    cc_token:="${CC_TOKEN:-}" \
    people:="${PEOPLE:-true}" \
    inspection:="${INSPECTION:-true}"

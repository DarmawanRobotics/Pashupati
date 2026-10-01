#!/bin/bash
# Usage: script/start_tmux.sh [mapping|patrol]
#   mapping: FAST-LIO saves the PCD map, record a route with the RViz panel
#   patrol:  map saving off, route loaded, navigation ready (start it from the panel)
set -euo pipefail

MODE="${1:-patrol}"
WS="${WS:-/home/robot/dev/Pashupati}"
ROUTE="${ROUTE:-$WS/map/example/example_waypoint.csv}"
TAGS="${TAGS:-$WS/map/example/tag_config.json}"
ROBOT_NS="${ROBOT_NS:-l1w}"
SESSION=pashupati

if [[ "$MODE" != "mapping" && "$MODE" != "patrol" ]]; then
    echo "usage: $0 [mapping|patrol]" >&2
    exit 1
fi
SAVE_MAP=$([[ "$MODE" == "mapping" ]] && echo true || echo false)

# Stop the vendor services that would fight for the robot
if command -v robot-launch >/dev/null; then
    robot-launch stop forward || true
fi
tmux kill-session -t "$SESSION" 2>/dev/null || true

SETUP="source /opt/ros/humble/setup.bash && source $WS/install/setup.bash && cd $WS"

run() {  # run <pane> <command>
    tmux send-keys -t "$SESSION:$1" "$SETUP && $2" C-m
}

# Window 1: robot, odometry, perception
tmux new-session -d -s "$SESSION" -n robot
tmux split-window -v -t "$SESSION:robot"
tmux split-window -h -t "$SESSION:robot.0"
tmux split-window -h -t "$SESSION:robot.2"
run robot.0 "ros2 launch l1w_bringup bringup.launch.py namespace:=$ROBOT_NS"
sleep 3
run robot.1 "ros2 launch robot_mapping mapping.launch.py save_map:=$SAVE_MAP tags_config_file:=$TAGS"
run robot.2 "ros2 launch robot_perception perception.launch.py"
run robot.3 "ros2 run rviz2 rviz2 -d \$(ros2 pkg prefix robot_rviz)/share/robot_rviz/rviz/robot.rviz \
--ros-args -r battery:=/$ROBOT_NS/battery -r cmd_vel:=/$ROBOT_NS/cmd_vel \
-r motor_temperature:=/$ROBOT_NS/motor_temperature -r diagnostics:=/$ROBOT_NS/diagnostics"
tmux select-layout -t "$SESSION:robot" tiled

# Window 2: navigation + command pane
tmux new-window -t "$SESSION" -n nav
tmux split-window -v -t "$SESSION:nav"
if [[ "$MODE" == "patrol" ]]; then
    run nav.0 "ros2 launch robot_navigation navigation.launch.py waypoints_file:=$ROUTE \
cmd_vel_topic:=/$ROBOT_NS/cmd_vel battery_topic:=/$ROBOT_NS/battery"
fi
tmux send-keys -t "$SESSION:nav.1" "$SETUP" C-m
tmux send-keys -t "$SESSION:nav.1" "# ros2 service call /localization/start std_srvs/srv/Trigger" C-m

tmux select-window -t "$SESSION:robot"
tmux attach-session -t "$SESSION"

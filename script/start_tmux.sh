#!/bin/bash

# Stop existing robot services
robot-launch stop forward

# Setup ROS 2 environment
source /opt/ros/humble/setup.bash
source /home/robot/dev/Pashupati/install/setup.bash
cd /home/robot/dev/Pashupati/


# ==========================================
# Panel 1: Description, Driver, Mapping, Perception
# ==========================================

tmux new-session -d -s pashupati -n panel1

# Robot description + RViz
tmux send-keys -t pashupati:panel1.0 \
    'ros2 launch robot_description description.launch.py use_rviz:=true' C-m

# Base driver
tmux split-window -v -t pashupati:panel1.0
tmux send-keys -t pashupati:panel1.1 \
    'ros2 launch robot_drivers base_driver.launch.py' C-m

# Mapping
tmux split-window -h -t pashupati:panel1.1
tmux send-keys -t pashupati:panel1.2 \
    'ros2 launch robot_mapping mapping.launch.py' C-m

# Perception
tmux split-window -h -t pashupati:panel1.0
tmux send-keys -t pashupati:panel1.3 \
    'ros2 launch robot_perception perception.launch.py' C-m

# Arrange panes
tmux select-layout -t pashupati:panel1 tiled


# ==========================================
# Panel 2: Navigation, Localization, SDK
# ==========================================

sleep 2

tmux new-window -t pashupati -n panel2

# Navigation
tmux send-keys -t pashupati:panel2.0 \
    'ros2 launch robot_navigation navigation.launch.py' C-m

# Localization
tmux split-window -v -t pashupati:panel2.0
tmux send-keys -t pashupati:panel2.1 \
    'ros2 launch robot_localization localization.launch.py' C-m

# SDK bridge
tmux split-window -h -t pashupati:panel2.1
tmux send-keys -t pashupati:panel2.2 \
    'ros2 launch robot_drivers sdk_bridge.launch.py' C-m

# Command pane
tmux split-window -h -t pashupati:panel2.0
tmux send-keys -t pashupati:panel2.3 \
    'echo "ros2 service call /localization/start std_srvs/srv/Trigger"' C-m

# Arrange panes
tmux select-layout -t pashupati:panel2 tiled


# ==========================================
# Attach Session
# ==========================================

tmux select-window -t pashupati:panel1
tmux attach-session -t pashupati
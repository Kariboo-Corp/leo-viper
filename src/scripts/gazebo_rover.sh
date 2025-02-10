#!/bin/bash

# Source ROS 2 workspace
source /home/ws/install/setup.bash

# Launch the Leo Rover simulation
ros2 launch leo_gz_bringup leo_gz.launch.py

#!/bin/bash
set -e

source /opt/humble/setup.bash

if [ -f /root/unitree_ros2_ws/install/setup.bash ]; then
	source /root/unitree_ros2_ws/install/setup.bash
fi

exec "$@"

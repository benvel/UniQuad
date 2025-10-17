# .devcontainer/ros_entrypoint.sh
#!/bin/bash
set -e

source /opt/ros/humble/setup.bash   # <-- corrected

if [ -f /root/unitree_ros2_ws/install/setup.bash ]; then
  source /root/unitree_ros2_ws/install/setup.bash
fi

exec "$@"

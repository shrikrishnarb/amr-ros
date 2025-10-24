#!/usr/bin/env bash
set -e
# Source ROS 2
source /opt/ros/humble/setup.bash || true
# Source your workspace if already built inside the container
if [ -f /workspace/colcon_ws/install/setup.bash ]; then
  source /workspace/colcon_ws/install/setup.bash
fi
# X11 workaround for shared memory
export QT_X11_NO_MITSHM=1
exec "$@"
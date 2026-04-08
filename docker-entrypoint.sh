#!/bin/bash
set -e

# Source ROS2 environment

if [ -z "${ROS_DISTRO:-}" ]; then
    echo "Error: ROS_DISTRO environment variable is not set. Cannot source /opt/ros/<distro>/setup.bash." >&2
    exit 1
fi

source /opt/ros/${ROS_DISTRO}/setup.bash
source /ros2_ws/install/setup.bash

exec "$@"

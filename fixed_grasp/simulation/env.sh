#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
source /workspace/devel/setup.bash
export ROS_PACKAGE_PATH="/workspace/simulation:${ROS_PACKAGE_PATH}"
export ROS_MASTER_URI=http://127.0.0.1:11321
exec "$@"

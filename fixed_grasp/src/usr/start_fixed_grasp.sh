#!/usr/bin/env bash
set -e
startup_args=("$@")
# ROS setup scripts interpret positional arguments of the calling shell.
set --
startup_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/noetic/setup.bash
source "$startup_root/devel/setup.bash"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
cd "$startup_root"
exec /usr/bin/python3 "$startup_root/src/usr/fixed_grasp/scripts/start_system.py" "${startup_args[@]}"

#!/usr/bin/env bash
# Source from the workspace; uses locally extracted official Noetic packages.
_mtc_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.." && pwd)"
source /opt/ros/noetic/setup.bash
source "$_mtc_ws/devel/setup.bash"
_mtc_prefix="$_mtc_ws/.dependencies/mtc/root/opt/ros/noetic"
export PYTHONPATH="$_mtc_prefix/lib/python3/dist-packages:$PYTHONPATH"
export LD_LIBRARY_PATH="$_mtc_prefix/lib:$LD_LIBRARY_PATH"
export ROS_PACKAGE_PATH="$_mtc_prefix/share:$ROS_PACKAGE_PATH"
export CMAKE_PREFIX_PATH="$_mtc_prefix:$CMAKE_PREFIX_PATH"
unset _mtc_ws _mtc_prefix

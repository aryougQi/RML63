#!/usr/bin/env bash
set -e
build_args=("$@")
set --
source /opt/ros/noetic/setup.bash
build_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$build_root"
exec catkin_make -j2 -l2 -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DPYTHON_EXECUTABLE=/usr/bin/python3 "${build_args[@]}"

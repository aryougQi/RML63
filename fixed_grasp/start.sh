#!/usr/bin/env bash
set -e
entry_args=("$@")
set --
entry_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/noetic/setup.bash
if [ ! -f "$entry_root/devel/setup.bash" ]; then
  echo "请先运行 bash build.sh 编译此工程。" >&2
  exit 1
fi
source "$entry_root/devel/setup.bash"
cd "$entry_root"
exec bash "$entry_root/src/usr/start_fixed_grasp.sh" "${entry_args[@]}"

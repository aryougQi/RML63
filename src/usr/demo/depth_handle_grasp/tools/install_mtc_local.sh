#!/usr/bin/env bash
set -euo pipefail
ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.." && pwd)"
mkdir -p "$ws/.dependencies/mtc"
cd "$ws/.dependencies/mtc"
apt-get download ros-noetic-moveit-task-constructor-core ros-noetic-moveit-task-constructor-msgs ros-noetic-moveit-task-constructor-visualization ros-noetic-rviz-marker-tools
for package in ./*.deb; do dpkg-deb -x "$package" root; done
sha256sum ./*.deb > SHA256SUMS

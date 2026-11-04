#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if ! docker image inspect rml63/noetic-sim:local >/dev/null 2>&1; then
  docker build -t rml63/noetic-sim:local -f simulation/Dockerfile.base .
fi
exec docker compose up -d --build "$@"

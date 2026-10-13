#!/usr/bin/env bash
# Start (or attach to) the control container on the robot. The workspace is the g1_ws that
# contains this package; build it inside the container with:
#   colcon build --symlink-install && source install/setup.bash
set -e
WS="$(cd "$(dirname "$0")/../../.." && pwd)"
NAME=g1_control

if docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
  exec docker exec -it "$NAME" bash
fi

# --net host:        see eth0 and Unitree's DDS traffic directly
# SYS_NICE / rtprio: lets the controller manager run its loop with real-time priority
exec docker run -it --rm --name "$NAME" \
  --net host \
  --cap-add SYS_NICE --ulimit rtprio=99 --ulimit memlock=-1 \
  -v "$WS":/g1_ws \
  -v /dev:/dev \
  --device-cgroup-rule 'c 81:* rmw' --device-cgroup-rule 'c 189:* rmw' \
  g1_control:latest bash

#!/usr/bin/env bash
# Build the control image on the robot. Run from anywhere.
set -e
cd "$(dirname "$0")"
cp ../config/cyclonedds_robot.xml .
docker build -t g1_control:latest .
rm cyclonedds_robot.xml

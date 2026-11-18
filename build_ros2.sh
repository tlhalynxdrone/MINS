#!/bin/bash
set -e

# MINS repo root (dir holding this script) and the colcon workspace root above src/.
MINS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${MINS_DIR}/../.."

source "/opt/ros/${ROS_DISTRO}/setup.bash"
# cv_bridge built from source (src/vision_opencv) so it links the same OpenCV 4.8 as MINS
colcon build --packages-select cv_bridge
source install/setup.bash
colcon build --paths "${MINS_DIR}"/thirdparty/*
source install/setup.bash
# NVIDIA's OpenCV 4.8 has no contrib modules, so aruco support is disabled
colcon build --paths "${MINS_DIR}"/thirdparty/open_vins/* --cmake-args -DENABLE_ARUCO_TAGS=OFF
source install/setup.bash
colcon build --paths "${MINS_DIR}"/mins "${MINS_DIR}"/mins_data
source install/setup.bash
colcon build --paths "${MINS_DIR}"/mins_eval

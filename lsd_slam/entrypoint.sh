#!/bin/bash

set -e

if [ -f /opt/ros/${ROS_DISTRO}/setup.sh ]
then
  source /opt/ros/${ROS_DISTRO}/setup.sh
else
  echo "/opt/ros/${ROS_DISTRO}/setup.sh is not provided. \`apk add --no-cache ros-${ROS_DISTRO}-catkin\` to setup."
fi

source /root/rosbuild_ws/setup.sh

exec "$@"

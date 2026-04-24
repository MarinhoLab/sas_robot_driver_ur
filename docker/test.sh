#!/bin/bash

sudo apt-get install tree

cd /root/sas_robot_driver_ur_devel
ls .
colcon build
source install/setup.bash

PACKAGE_SHARE_PATH=$(ros2 pkg prefix sas_robot_driver_ur --share)

echo "$PACKAGE_SHARE_PATH"
tree "$PACKAGE_SHARE_PATH"
cat robots/ur3e.json

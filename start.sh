#!/bin/bash

source /opt/ros/humble/setup.bash
source ~/uros_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=169

#if [ $? -ne 0 ]; then
#    echo "Failed to source ROS 2 setup.bash"
#    exit 1
#fi

sleep 3
ros2 launch diadem_firmware bringup.launch.py

#if [ $? -ne 0 ]; then
#    echo "Failed to launch autobringup.launch.py"
#    exit 1
#fi



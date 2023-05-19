#!/bin/bash

. /opt/ros/foxy/setup.bash
cd /home/dockeruser/dev_ws/pub_sub_custom_msg
. install/setup.bash

ros2 launch pub_sub_custom_msg pid_launch.py 

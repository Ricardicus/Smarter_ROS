#!/bin/bash

cd /home/dockeruser/dev_ws
source /opt/ros/foxy/setup.bash
source ./install/setup.bash
source ./pub_sub_custom_msg/install/setup.bash

./pub_sub_custom_msg/install/pub_sub_custom_msg/lib/pub_sub_custom_msg/pid_controller

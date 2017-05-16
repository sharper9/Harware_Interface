#!/bin/bash

source /opt/ros/indigo/setup.bash
source ~/rmc_ws/devel/setup.bash

gnome-terminal \
	--tab -e "bash -c 'roslaunch hw_interface hw_interface_operator_only.launch'" --title="hw_interface" \
	--tab -e "bash -c 'sleep 2 && rosrun joy joy_node'" --title="joy"

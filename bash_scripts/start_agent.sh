#!/bin/bash

source /opt/ros/indigo/setup.bash
source ~/rmc_ws/devel/setup.bash

gnome-terminal \
	--tab -e "bash -c 'roslaunch hw_interface hw_interface.launch'" --title="hw_interface" \
	--tab -e "bash -c 'sleep 2 && rosrun tele_op_control tele_op_control_node'" --title="tele_op" \
	--tab -e "bash -c 'sleep 2 && rosrun td_navigation td_navigation'" --title="td_nav" \
	--tab -e "bash -c 'sleep 2 && rosrun navigation navigation_filter_node'" --title="nav_filter"\
	--tab -e "bash -c 'sleep 2 && rosrun robot_control exec_node'" --title="exec" \
	--tab -e "bash -c 'sleep 2 && rosrun robot_control mission_planning_node'" --title="mission_planning"

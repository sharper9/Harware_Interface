#!/bin/bash

source /opt/ros/indigo/setup.bash
source ~/rmc_ws/devel/setup.bash

gnome-terminal \
	--tab -e "bash -c 'roslaunch hw_interface hw_interface.launch'" \
	--tab -e "bash -c 'sleep 2 && rosrun tele_op_control tele_op_control_node'" \
	--tab -e "bash -c 'sleep 2 && rosrun td_navigation td_navigation'" \
	--tab -e "bash -c 'sleep 2 && rosrun navigation navigation_filter_node'" \
	--tab -e "bash -c 'sleep 2 && rosrun robot_control exec_node'" \
	--tab -e "bash -c 'sleep 2 && rosrun robot_control mission_planning_node'" 

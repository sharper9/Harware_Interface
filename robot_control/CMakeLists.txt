cmake_minimum_required(VERSION 2.8.3)
project(robot_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  messages
  geometry_msgs
  sensor_msgs
  hw_interface_plugin_roboteq
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   DriveSpeeds.msg
   Waypoint.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES robot_control
  CATKIN_DEPENDS message_generation message_runtime roscpp rospy
#  DEPENDS system_lib
)

add_definitions("-std=gnu++11")

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
 add_library(exec src/exec.cpp)
 add_library(action src/action.cpp)
 add_library(idle src/idle.cpp)
 add_library(drive_global src/drive_global.cpp)
 add_library(drive_relative src/drive_relative.cpp)
 add_library(drive_to_wall src/drive_to_wall.cpp)
 add_library(dig src/dig.cpp)
 add_library(dump src/dump.cpp)
 add_library(wait src/wait.cpp)
 add_library(raise_arm src/raise_arm.cpp)
 add_library(partially_raise_arm src/partially_raise_arm.cpp)
 add_library(raise_bucket src/raise_bucket.cpp)
 add_library(lower_bucket src/lower_bucket.cpp)
 add_library(prepare_arm_bucket src/prepare_arm_bucket.cpp)
 add_library(shake_action src/shake.cpp)
 add_library(drive_halt src/drive_halt.cpp)
 add_library(drive_straight src/drive_straight.cpp)
 add_library(drive_until_limit src/drive_until_limit.cpp)
 add_library(drive_pivot src/drive_pivot.cpp)
 add_library(scoop_halt src/scoop_halt.cpp)
 add_library(scoop_set_pos src/scoop_set_pos.cpp)
 add_library(arm_halt src/arm_halt.cpp)
 add_library(arm_set_pos src/arm_set_pos.cpp)
 add_library(arm_lower_to_pitch src/arm_lower_to_pitch.cpp)
 add_library(arm_shake src/arm_shake.cpp)
 add_library(bucket_halt src/bucket_halt.cpp)
 add_library(bucket_set_pos src/bucket_set_pos.cpp)
 add_library(bucket_shake src/bucket_shake.cpp)

 add_library(mission_planning src/mission_planning.cpp)
 add_library(initialize src/initialize.cpp)
 add_library(drive_to_dig src/drive_to_dig.cpp)
 add_library(mine src/mine.cpp)
 add_library(drive_to_deposit src/drive_to_deposit.cpp)
 add_library(deposit_realign src/deposit_realign.cpp)
 add_library(deposit src/deposit.cpp)
 add_library(recover src/recover.cpp)
 add_library(flip_back src/flip_back.cpp)
 add_library(pause src/pause.cpp)

## Declare a cpp executable
 add_executable(exec_node src/exec_node.cpp)
 add_executable(mission_planning_node src/mission_planning_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
 add_dependencies(exec_node robot_control_generate_messages_cpp)
 add_dependencies(mission_planning_node robot_control_generate_messages_cpp)

## Specify libraries to link a library or executable target against

 target_link_libraries(exec_node
   ${catkin_LIBRARIES}
   exec
   action
   idle
   drive_global
   drive_relative
   drive_to_wall
   dig
   dump
   wait
   raise_arm
   partially_raise_arm
   raise_bucket
   lower_bucket
   prepare_arm_bucket
   shake_action
   drive_halt
   drive_straight
   drive_until_limit
   drive_pivot
   scoop_halt
   scoop_set_pos
   arm_halt
   arm_set_pos
   arm_lower_to_pitch
   arm_shake
   bucket_halt
   bucket_set_pos
   bucket_shake
   
 )

 target_link_libraries(mission_planning_node
   ${catkin_LIBRARIES}
   mission_planning
   initialize
   drive_to_dig
   mine
   drive_to_deposit
   deposit_realign
   deposit
   recover
   flip_back
   pause
 )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS robot_control robot_control_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_robot_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

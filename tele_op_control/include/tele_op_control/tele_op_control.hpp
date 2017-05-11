#ifndef TELE_OP_CONTROL_HPP
#define TELE_OP_CONTROL_HPP

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <armadillo>

#include <messages/ActuatorOut.h>
#include <messages/ExecInfo.h>

#include <hw_interface_plugin_agent/pause.h>
#include <robot_control/bit_utils.h>

#define ROBOT_RANGE 1000 // -1000 to 1000
#define JOYSTICK_DEADBAND 0.2

// button indexes
#define A_INDEX 0
#define B_INDEX 1
#define X_INDEX 2
#define Y_INDEX 3
#define LB_INDEX 4
#define RB_INDEX 5
#define BACK_INDEX 6
#define START_INDEX 7

// left joystick axes indexes
#define L_LEFT_RIGHT_INDEX 0
#define L_UP_DOWN_INDEX 1

// left trigger
#define LT_INDEX 2 // 1 to -1

// right joystick axes indexes
#define R_LEFT_RIGHT_INDEX 3
#define R_UP_DOWN_INDEX 4

// right trigger
#define RT_INDEX 5 // 1 to -1

#define CROSS_LEFT_RIGHT_INDEX 6 // left = 1; right = -1
#define CROSS_UP_DOWN_INDEX 7 // up = 1; down = -1

class TeleOp
{
public:
  TeleOp();
  ros::NodeHandle nh;

private:
  ros::Publisher actuator_pub_;
  ros::Subscriber joystick_sub_;

  ros::Publisher pause_robot_control_pub_;
  hw_interface_plugin_agent::pause pause_msg_;
  Toggle toggle;

  void joystickCallback(const sensor_msgs::JoyConstPtr &msg);
};

#endif // TELE_OP_CONTROL_HPP

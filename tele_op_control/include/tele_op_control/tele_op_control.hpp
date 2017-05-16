#ifndef TELE_OP_CONTROL_HPP
#define TELE_OP_CONTROL_HPP

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <armadillo>

#include <messages/ActuatorOut.h>
#include <messages/ExecInfo.h>
#include <messages/ExecManualOverride.h>

#include <hw_interface_plugin_agent/pause.h>
#include <robot_control/bit_utils.h>

#define ROBOT_RANGE 1000 // -1000 to 1000
#define JOYSTICK_DEADBAND 0.2

#define WRIST_RAISED 1000
#define WRIST_LOWERED -1000
#define WRIST_OFFSET 15

#define ARM_RAISED 1000
#define ARM_LOWERED -1000
#define ARM_OFFSET 15
#define ARM_MAX_BUCKET_RAISED 535

#define BUCKET_RAISED 1000
#define BUCKET_LOWERED -1000
#define BUCKET_BUMP 850

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
  ros::Subscriber actuator_sub_;
  bool init_manual_override_;
  bool manual_override_;
  
  ros::Subscriber joystick_sub_;
  ros::Publisher exec_info_pub_;

  ros::ServiceClient exec_manual_override_client_;
  ros::Publisher pause_robot_control_pub_;
  hw_interface_plugin_agent::pause pause_msg_;
  messages::ExecManualOverride exec_manual_override_srv_;
  Toggle pause_toggle_;
  Toggle manual_override_toggle_;
  Leading_Edge_Latch manual_override_latch_;
  
  int bucket_pos_, wrist_pos_, arm_pos_;

  void joystickCallback(const sensor_msgs::JoyConstPtr &msg);
  void manualOverrideCallback(const messages::ActuatorOut::ConstPtr &msg);
};

#endif // TELE_OP_CONTROL_HPP

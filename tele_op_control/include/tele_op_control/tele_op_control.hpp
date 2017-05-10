#ifndef TELE_OP_CONTROL_HPP
#define TELE_OP_CONTROL_HPP

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <armadillo>

#include <messages/ActuatorOut.h>
#include <messages/ExecInfo.h>

#define ROBOT_RANGE 1000 // -1000 to 1000
#define JOYSTICK_DEADBAND 0.2

class TeleOp
{
public:
  TeleOp();
  ros::NodeHandle nh;

private:
  ros::Publisher actuator_pub_;
  ros::Subscriber joystick_sub_;

  void joystickCallback(const sensor_msgs::JoyConstPtr &msg);
};

#endif // TELE_OP_CONTROL_HPP

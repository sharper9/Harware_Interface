/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
// %EndTag(MSG_HEADER)%
#include <hw_interface_plugin_dynamixel/servoUpdate.h>
#include <hw_interface_plugin_dynamixel/groupServoUpdate.h>
#include <hw_interface_plugin_dynamixel/servoStateResponse.h>
#include <sstream>


#define PI 3.14159265359
#define DEG2RAD PI/180.0
#define RAD2DEG 180.0/PI

#define TURNING_RANGE 4095.0
#define SPEED_RANGE 2047.0
#define TURN_MAX_RANGE 360.0
#define ROBOT_MAX_SPEED 0.34 // m/s
#define ROBOT_MAX_ANG_SPEED 120.0 // deg/s
#define ROBOT_WHEEL_R 0.065 // m
#define WHEEL_MAX_ANG_SPEED 5.23598775 // rad/s

#define FL_TURN_OFFSET 2048.0
#define BL_TURN_OFFSET 2048.0
#define BR_TURN_OFFSET 2048.0
#define FR_TURN_OFFSET 2048.0
#define SPEED_OFFSET 1024.0

#define NUM_SERVOS 8

#define FL_TURN_ID 2
#define BL_TURN_ID 3
#define BR_TURN_ID 4
#define FR_TURN_ID 1
#define FL_DRIVE_ID 6
#define BL_DRIVE_ID 7
#define BR_DRIVE_ID 8
#define FR_DRIVE_ID 5

void state1callback(const hw_interface_plugin_dynamixel::servoStateResponse::ConstPtr &msg)
{
    if(msg->queryAbility==32768)
    {
        ROS_INFO_THROTTLE(0.5, "Present Current %u", msg->uDATA);
        ROS_INFO_THROTTLE(0.5, "Present Torque %f", msg->uDATA/33000.0*2048);
    }
}

int main(int argc, char **argv)
{
enum SERVO_INDEX_T {_flTurn, _blTurn, _brTurn, _frTurn, _flDrive, _blDrive, _brDrive, _frDrive};
  ros::init(argc, argv, "talker");

  ros::NodeHandle nh;

  ros::Subscriber state1sub = nh.subscribe<hw_interface_plugin_dynamixel::servoStateResponse>("/servo/state/1", 1, state1callback);

  int val = 1000;
  ros::spin();


  return 0;
}
// %EndTag(FULLTEXT)%

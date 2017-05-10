#include <tele_op_control/tele_op_control.hpp>

TeleOp::TeleOp()
{
  actuator_pub_  = nh.advertise<messages::ActuatorOut>("/control/actuatorout/all", 1);
  joystick_sub_ = nh.subscribe<sensor_msgs::Joy>("/joy", 1, &TeleOp::joystickCallback, this);
}

void TeleOp::joystickCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
  messages::ActuatorOut actuator;
  float joystickMagnitude = hypot(msg->axes[3], msg->axes[4]);
  if (joystickMagnitude > 1.0) joystickMagnitude = 1.0;

  if (msg->axes[7] == -1.0) //arm down (crosspad down)
  {
    actuator.arm_pos_cmd = -ROBOT_RANGE;
  }
  else if (msg->axes[7] == 1.0) //arm up (crosspad up)
  {
    actuator.arm_pos_cmd = ROBOT_RANGE;
  }
  else if (!msg->axes[7])
  {
    actuator.arm_pos_cmd = 0;
  }

  if (msg->axes[5] == -1) //wrist down (RT)
  {
    actuator.wrist_pos_cmd = -ROBOT_RANGE;
  }
  else if (msg->axes[2] == -1) //wrist up (LT)
  {
    actuator.wrist_pos_cmd = ROBOT_RANGE;
  }
  else if (msg->axes[2] == 1 && msg->axes[5] == 1)
  {
    actuator.wrist_pos_cmd = 0;
  }

  if (msg->buttons[5] == 1) //bucket down (RB)
  {
    actuator.bucket_pos_cmd = -ROBOT_RANGE;
  }
  else if (msg->buttons[4] == 1) //bucket up (LB)
  {
    actuator.bucket_pos_cmd = ROBOT_RANGE;
  }
  else if (!msg->buttons[4] && !msg->buttons[5])
  {
    actuator.bucket_pos_cmd = 0;
  }

  if(joystickMagnitude < JOYSTICK_DEADBAND)
  {
    actuator.fl_speed_cmd = 0.0;
    actuator.fr_speed_cmd = 0.0;
    actuator.bl_speed_cmd = 0.0;
    actuator.br_speed_cmd = 0.0;
  }
  else
  {
    float speed = ROBOT_RANGE*pow(joystickMagnitude, 3.0);

    if (msg->axes[4] >= JOYSTICK_DEADBAND) //forward
    {
      actuator.fl_speed_cmd = speed;
      actuator.fr_speed_cmd = speed;
      actuator.bl_speed_cmd = speed;
      actuator.br_speed_cmd = speed;

      if (msg->axes[0] >= JOYSTICK_DEADBAND) //left
      {
        actuator.fr_speed_cmd = -speed;
        actuator.br_speed_cmd = -speed;
      }
      else if (msg->axes[0] <= -JOYSTICK_DEADBAND) //right
      {
        actuator.fl_speed_cmd = -speed;
        actuator.bl_speed_cmd = -speed;
      }

    }

    else if (msg->axes[4] <= -JOYSTICK_DEADBAND)
    {
      actuator.fl_speed_cmd = -speed;
      actuator.fr_speed_cmd = -speed;
      actuator.bl_speed_cmd = -speed;
      actuator.br_speed_cmd = -speed;

      if (msg->axes[0] >= JOYSTICK_DEADBAND)
      {
        actuator.fl_speed_cmd = speed;
        actuator.bl_speed_cmd = speed;
      }
      else if (msg->axes[0] <= -JOYSTICK_DEADBAND)
      {
        actuator.fr_speed_cmd = speed;
        actuator.br_speed_cmd = speed;
      }

    }

  }
  actuator_pub_.publish(actuator);

}

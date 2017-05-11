#include <tele_op_control/tele_op_control.hpp>

TeleOp::TeleOp()
{
  actuator_pub_  = nh.advertise<messages::ActuatorOut>("/control/actuatorout/all", 1);
  joystick_sub_ = nh.subscribe<sensor_msgs::Joy>("/joy", 1, &TeleOp::joystickCallback, this);

  pause_robot_control_pub_ = nh.advertise<hw_interface_plugin_agent::pause>("/agent/pause", 1);

  pause_msg_.pause = false;
  toggle.toggle(0);
}

void TeleOp::joystickCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
  if (toggle.toggle(msg->buttons[START_INDEX]))
  {
    pause_msg_.pause = true;
    pause_robot_control_pub_.publish(pause_msg_);
  }
  else
  {
    pause_msg_.pause = false;
    pause_robot_control_pub_.publish(pause_msg_);
  }


  messages::ActuatorOut actuator;
  float joystickMagnitude = hypot(msg->axes[0], msg->axes[1]);
  if (joystickMagnitude > 1.0) joystickMagnitude = 1.0;

  // TODO: fix bucket for closed loop
  if (msg->buttons[Y_INDEX]) //bucket up
  {
    actuator.bucket_pos_cmd = -ROBOT_RANGE;
  }
  else if (msg->buttons[X_INDEX]) //bucket stow
  {
    actuator.bucket_pos_cmd = ROBOT_RANGE;
  }
  else if (msg->buttons[B_INDEX]) //bucket bump
  {
    actuator.bucket_pos_cmd = 0;
  }

  // TODO: fix arm for closed loop
  if (msg->axes[RT_INDEX] == -1) //arm up (RT)
  {
    actuator.arm_pos_cmd = ROBOT_RANGE;
  }
  else if (msg->axes[LT_INDEX] == -1) //arm down (LT)
  {
    actuator.arm_pos_cmd = -ROBOT_RANGE;
  }
  else if (msg->axes[2] && msg->axes[5])
  {
    actuator.arm_pos_cmd = 0;
  }

  // TODO: fix wrist for closed loop
  if (msg->buttons[RB_INDEX]) //wrist forward (RB)
  {
    actuator.wrist_pos_cmd = ROBOT_RANGE;
  }
  else if (msg->buttons[LB_INDEX]) //wrist backward (LB)
  {
    actuator.wrist_pos_cmd = -ROBOT_RANGE;
  }
  else if (!msg->buttons[RB_INDEX] && !msg->buttons[LB_INDEX])
  {
    actuator.wrist_pos_cmd = 0;
  }
  else if (msg->buttons[A_INDEX]) //reset wrist posiiton
  {
    actuator.wrist_pos_cmd = 0;
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

    if (msg->axes[L_UP_DOWN_INDEX] >= JOYSTICK_DEADBAND) //forward
    {
      actuator.fl_speed_cmd = speed;
      actuator.fr_speed_cmd = speed;
      actuator.bl_speed_cmd = speed;
      actuator.br_speed_cmd = speed;

      if (msg->axes[R_LEFT_RIGHT_INDEX] >= JOYSTICK_DEADBAND) //left
      {
        actuator.fl_speed_cmd = -speed;
        actuator.bl_speed_cmd = -speed;
      }
      else if (msg->axes[R_LEFT_RIGHT_INDEX] <= -JOYSTICK_DEADBAND) //right
      {
        actuator.fr_speed_cmd = -speed;
        actuator.br_speed_cmd = -speed;
      }

    }

    else if (msg->axes[L_UP_DOWN_INDEX] <= -JOYSTICK_DEADBAND) //reverse
    {
      actuator.fl_speed_cmd = -speed;
      actuator.fr_speed_cmd = -speed;
      actuator.bl_speed_cmd = -speed;
      actuator.br_speed_cmd = -speed;

      if (msg->axes[R_LEFT_RIGHT_INDEX] >= JOYSTICK_DEADBAND) //left
      {
        actuator.fl_speed_cmd = speed;
        actuator.bl_speed_cmd = speed;
      }
      else if (msg->axes[R_LEFT_RIGHT_INDEX] <= -JOYSTICK_DEADBAND) //right
      {
        actuator.fr_speed_cmd = speed;
        actuator.br_speed_cmd = speed;
      }

    }

  }
  actuator_pub_.publish(actuator);

}

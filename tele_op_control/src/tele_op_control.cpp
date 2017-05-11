#include <tele_op_control/tele_op_control.hpp>

TeleOp::TeleOp()
{
  actuator_pub_  = nh.advertise<messages::ActuatorOut>("/control/actuatorout/all", 1);
  joystick_sub_ = nh.subscribe<sensor_msgs::Joy>("/joy", 1, &TeleOp::joystickCallback, this);

  exec_manual_override_client_ = nh.serviceClient<messages::ExecManualOverride>("/control/exec/manualoverride");
  pause_robot_control_pub_ = nh.advertise<hw_interface_plugin_agent::pause>("/agent/pause", 1);

  pause_msg_.pause = true;
  exec_manual_override_srv_.request.manualOverride = false;
  pause_toggle_.toggle(0);
  pause_toggle_.toggle(1);
  pause_toggle_.toggle(0); // Set pause toggle to true to start
  manual_override_toggle_.toggle(0);
  manual_override_latch_.LE_Latch(0);
}

void TeleOp::joystickCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
  if (pause_toggle_.toggle(msg->buttons[START_INDEX]))
  {
    ROS_WARN("PAUSE");
    pause_msg_.pause = true;
    pause_robot_control_pub_.publish(pause_msg_);
  }
  else
  {
    ROS_WARN("UNPAUSE");
    pause_msg_.pause = false;
    pause_robot_control_pub_.publish(pause_msg_);
  }

  if (manual_override_latch_.LE_Latch(msg->buttons[BACK_INDEX]))
  {
    if (exec_manual_override_srv_.request.manualOverride)
    {
      ROS_WARN("ENAGAE MANUAL OVERRIDE");
      exec_manual_override_srv_.request.manualOverride = false;
      exec_manual_override_client_.call(exec_manual_override_srv_);
    }
    else
    {
      ROS_WARN("DISENAGAE MANUAL OVERRIDE");
      exec_manual_override_srv_.request.manualOverride = true;
      exec_manual_override_client_.call(exec_manual_override_srv_);
    }
  }

  messages::ActuatorOut actuator;
  float joystickMagnitude = hypot(msg->axes[0], msg->axes[1]);
  if (joystickMagnitude > 1.0) joystickMagnitude = 1.0;

  // TODO: fix bucket for closed loop
  if (msg->buttons[Y_INDEX]) //bucket up
  {
    actuator.bucket_pos_cmd = BUCKET_RAISED;
  }
  else if (msg->buttons[X_INDEX]) //bucket stow
  {
    actuator.bucket_pos_cmd = BUCKET_LOWERED;
  }
  else if (msg->buttons[B_INDEX]) //bucket bump
  {
    actuator.bucket_pos_cmd = 0;
  }

  // TODO: fix arm for closed loop
  if (msg->axes[RT_INDEX] == -1) //arm up (RT)
  {
    actuator.arm_pos_cmd = ARM_RAISED;
  }
  else if (msg->axes[LT_INDEX] == -1) //arm down (LT)
  {
    actuator.arm_pos_cmd = ARM_LOWERED;
  }
  else if (msg->axes[2] && msg->axes[5])
  {
    actuator.arm_pos_cmd = 0;
  }

  // TODO: fix wrist for closed loop
  if (msg->buttons[RB_INDEX]) //wrist forward (RB)
  {
    actuator.wrist_pos_cmd = SCOOP_LOWERED;
  }
  else if (msg->buttons[LB_INDEX]) //wrist backward (LB)
  {
    actuator.wrist_pos_cmd = SCOOP_RAISED;
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
  if(exec_manual_override_srv_.request.manualOverride)
  {
    actuator_pub_.publish(actuator);
  }

}

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
  
  bucket_pos_ = -1000;
  arm_pos_ = 0;
  wrist_pos_ = 0;
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
      exec_manual_override_srv_.request.manualOverride = true;
      exec_manual_override_client_.call(exec_manual_override_srv_);
    }
    else
    {
      ROS_WARN("DISENAGAE MANUAL OVERRIDE");
      exec_manual_override_srv_.request.manualOverride = false;
      exec_manual_override_client_.call(exec_manual_override_srv_);
    }
  }

  messages::ActuatorOut actuator;
  float joystickMagnitude = hypot(msg->axes[0], msg->axes[1]);
  if (joystickMagnitude > 1.0) joystickMagnitude = 1.0;

  if (msg->buttons[Y_INDEX]) //bucket up
  {
    bucket_pos_ = BUCKET_RAISED;
  }
  else if (msg->buttons[X_INDEX]) //bucket stow
  {
    bucket_pos_ = BUCKET_LOWERED;
  }
  else if (msg->buttons[B_INDEX]) //bucket bump
  {
    bucket_pos_ = BUCKET_BUMP;
  }

  if (msg->axes[RT_INDEX] == -1) //arm up (RT)
  {
    arm_pos_ += ARM_OFFSET;
    arm_pos_ = (arm_pos_ > ARM_RAISED) ? ARM_RAISED : arm_pos_;
  }
  else if (msg->axes[LT_INDEX] == -1) //arm down (LT)
  {
    arm_pos_ -= ARM_OFFSET;
    arm_pos_ = (arm_pos_ < ARM_LOWERED) ? ARM_LOWERED : arm_pos_;
  }

  if (msg->buttons[RB_INDEX]) //wrist forward (RB)
  {
    wrist_pos_ += WRIST_OFFSET;
    wrist_pos_ = (wrist_pos_ > WRIST_RAISED) ? WRIST_RAISED : wrist_pos_; 
  }
  else if (msg->buttons[LB_INDEX]) //wrist backward (LB)
  {
    wrist_pos_ -= WRIST_OFFSET;
    wrist_pos_ = (wrist_pos_ < WRIST_LOWERED) ? WRIST_LOWERED : wrist_pos_;
  }
  else if (msg->buttons[A_INDEX]) //reset wrist posiiton
  {
    wrist_pos_ = 0;
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
    actuator.bucket_pos_cmd = bucket_pos_;
    actuator_pub_.publish(actuator);
  }

}

#include <tele_op_control/tele_op_control.hpp>

TeleOp::TeleOp()
{
  actuator_sub_ = nh.subscribe<messages::ActuatorOut>("/control/actuatorout/all", 1, &TeleOp::manualOverrideCallback, this);
  actuator_pub_  = nh.advertise<messages::ActuatorOut>("/control/actuatorout/all", 1);
  joystick_sub_ = nh.subscribe<sensor_msgs::Joy>("/joy", 1, &TeleOp::joystickCallback, this);
  exec_manual_override_client_ = nh.serviceClient<messages::ExecManualOverride>("/control/exec/manualoverride");
  pause_robot_control_pub_ = nh.advertise<hw_interface_plugin_agent::pause>("/agent/pause", 1);

  exec_info_pub_ = nh.advertise<messages::ExecInfo>("/control/exec/info", 1);

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
  init_manual_override_ = false;
}

void TeleOp::joystickCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
  if(msg->axes.size() >= 5 && msg->buttons.size() >= 7)
  {
    if (pause_toggle_.toggle(msg->buttons[START_INDEX]))
    {
      ROS_WARN_THROTTLE(2, "PAUSE");
      pause_msg_.pause = true;
      pause_robot_control_pub_.publish(pause_msg_);
    }
    else
    {
      ROS_WARN_THROTTLE(2, "UNPAUSE");
      pause_msg_.pause = false;
      pause_robot_control_pub_.publish(pause_msg_);
    }

    if (manual_override_latch_.LE_Latch(msg->buttons[BACK_INDEX]))
    {
      if (exec_manual_override_srv_.request.manualOverride)
      {
        ROS_WARN_THROTTLE(2, "DISENAGAE MANUAL OVERRIDE");
        exec_manual_override_srv_.request.manualOverride = false;
        exec_manual_override_client_.call(exec_manual_override_srv_);
      }
      else
      {
        ROS_WARN_THROTTLE(2, "ENGAGE MANUAL OVERRIDE");
        exec_manual_override_srv_.request.manualOverride = true;
        exec_manual_override_client_.call(exec_manual_override_srv_);
        init_manual_override_ = true;
      }
    }
    
    if (!exec_manual_override_srv_.request.manualOverride) return;

    if (msg->buttons[Y_INDEX] && !(arm_pos_ >= ARM_MAX_BUCKET_RAISED) ) //bucket up
    {
      bucket_pos_ = BUCKET_RAISED;
    }
    else if (msg->buttons[X_INDEX]&& !(arm_pos_ >= ARM_MAX_BUCKET_RAISED)) //bucket stow
    {
      bucket_pos_ = BUCKET_LOWERED;
    }
    else if (msg->buttons[B_INDEX] && !(arm_pos_ >= ARM_MAX_BUCKET_RAISED)) //bucket bump
    {
      bucket_pos_ = BUCKET_BUMP;
    }

    if (msg->axes[RT_INDEX] == -1) //arm up (RT)
    {
        if (bucket_pos_ == BUCKET_LOWERED || (bucket_pos_ != BUCKET_LOWERED && !((arm_pos_ + ARM_OFFSET) > ARM_MAX_BUCKET_RAISED)))
        {
            arm_pos_ += ARM_OFFSET;
            arm_pos_ = (arm_pos_ > ARM_RAISED) ? ARM_RAISED : arm_pos_;
        }
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
    else if (msg->buttons[A_INDEX]) //reset wrist and arm posiiton
    {
      arm_pos_ = 0;
      wrist_pos_ = 0;
    }

    messages::ActuatorOut actuator;
    messages::ExecInfo exec_info_msg;

    // left joystick
    float joystickMagnitude = hypot(msg->axes[0], msg->axes[1]);
    if (joystickMagnitude > 1.0) joystickMagnitude = 1.0;

    if(joystickMagnitude < JOYSTICK_DEADBAND)
    {
      actuator.fl_speed_cmd = 0.0;
      actuator.fr_speed_cmd = 0.0;
      actuator.bl_speed_cmd = 0.0;
      actuator.br_speed_cmd = 0.0;
      exec_info_msg.stopFlag = true;
    }
    else if (msg->axes[R_LEFT_RIGHT_INDEX] != -1.0 || msg->axes[R_LEFT_RIGHT_INDEX] != 1.0)// left joystick
    {
      float speed = ROBOT_RANGE*pow(joystickMagnitude, 3.0);
      exec_info_msg.stopFlag = false;
      if (msg->axes[L_UP_DOWN_INDEX] >= JOYSTICK_DEADBAND) //forward
      {
        actuator.fl_speed_cmd = speed;
        actuator.fr_speed_cmd = speed;
        actuator.bl_speed_cmd = speed;
        actuator.br_speed_cmd = speed;
      }

      else if (msg->axes[L_UP_DOWN_INDEX] <= -JOYSTICK_DEADBAND) //reverse
      {
        actuator.fl_speed_cmd = -speed;
        actuator.fr_speed_cmd = -speed;
        actuator.bl_speed_cmd = -speed;
        actuator.br_speed_cmd = -speed;
      }
    }

    // right joystick
    float rightJoyMagnitude = hypot(msg->axes[R_LEFT_RIGHT_INDEX], msg->axes[R_UP_DOWN_INDEX]);
    if (rightJoyMagnitude > 1.0) rightJoyMagnitude = 1.0;

    if (!(rightJoyMagnitude < JOYSTICK_DEADBAND))
    {
      float speed = ROBOT_RANGE*pow(rightJoyMagnitude, 3.0);
      exec_info_msg.stopFlag = false;
      exec_info_msg.turnFlag = true;

      if (msg->axes[R_LEFT_RIGHT_INDEX] >= JOYSTICK_DEADBAND && msg->axes[R_UP_DOWN_INDEX] >= JOYSTICK_DEADBAND ) //forward left
      {
        actuator.fl_speed_cmd -= speed;
        actuator.fr_speed_cmd += speed;
        actuator.bl_speed_cmd -= speed;
        actuator.br_speed_cmd += speed;
      }
      else if (msg->axes[R_LEFT_RIGHT_INDEX] >= JOYSTICK_DEADBAND && msg->axes[R_UP_DOWN_INDEX] <= -JOYSTICK_DEADBAND ) //reverse left
      {
        actuator.fl_speed_cmd += speed;
        actuator.fr_speed_cmd -= speed;
        actuator.bl_speed_cmd += speed;
        actuator.br_speed_cmd -= speed;
      }
      else if (msg->axes[R_LEFT_RIGHT_INDEX] >= JOYSTICK_DEADBAND) //left
      {
        actuator.fl_speed_cmd -= speed;
        actuator.fr_speed_cmd += speed;
        actuator.bl_speed_cmd -= speed;
        actuator.br_speed_cmd += speed;
      }
      else if (msg->axes[R_LEFT_RIGHT_INDEX] <= -JOYSTICK_DEADBAND && msg->axes[R_UP_DOWN_INDEX] >= JOYSTICK_DEADBAND) //forward right
      {
        actuator.fl_speed_cmd += speed;
        actuator.fr_speed_cmd -= speed;
        actuator.bl_speed_cmd += speed;
        actuator.br_speed_cmd -= speed;
      }
      else if (msg->axes[R_LEFT_RIGHT_INDEX] <= -JOYSTICK_DEADBAND && msg->axes[R_UP_DOWN_INDEX] <= -JOYSTICK_DEADBAND) //reverse right
      {
        actuator.fl_speed_cmd -= speed;
        actuator.fr_speed_cmd += speed;
        actuator.bl_speed_cmd -= speed;
        actuator.br_speed_cmd += speed;
      }
      else if (msg->axes[R_LEFT_RIGHT_INDEX] <= -JOYSTICK_DEADBAND) //right
      {
        actuator.fl_speed_cmd += speed;
        actuator.fr_speed_cmd -= speed;
        actuator.bl_speed_cmd += speed;
        actuator.br_speed_cmd -= speed;
      }

    }
    
    if(msg->axes[CROSS_UP_DOWN_INDEX]>=0.95) //up
    {
        arm_pos_=ARM_RAISED;
    }
    else if(msg->axes[CROSS_UP_DOWN_INDEX]<=-0.95) //down
    {
        arm_pos_=-900;
    }
    
    if(msg->axes[CROSS_LEFT_RIGHT_INDEX]>=0.95)  //left
    {
        wrist_pos_=300;
    }
    else if(msg->axes[CROSS_LEFT_RIGHT_INDEX]<=-0.95)  //right
    {
        wrist_pos_=-1000;
    }

    if(exec_manual_override_srv_.request.manualOverride)
    {
      actuator.bucket_pos_cmd = bucket_pos_;
      actuator.arm_pos_cmd = arm_pos_;
      actuator.wrist_pos_cmd = wrist_pos_;
      actuator_pub_.publish(actuator);
      exec_info_pub_.publish(exec_info_msg);
    }
  }
}

void TeleOp::manualOverrideCallback(const messages::ActuatorOut::ConstPtr &msg)
{
    messages::ActuatorOut actuator;
    if (init_manual_override_)
    {
      actuator.bucket_pos_cmd = msg->bucket_pos_cmd;
      actuator.arm_pos_cmd = msg->arm_pos_cmd;
      actuator.wrist_pos_cmd = msg->wrist_pos_cmd;
      actuator_pub_.publish(actuator);
      init_manual_override_ = false;
    }
}

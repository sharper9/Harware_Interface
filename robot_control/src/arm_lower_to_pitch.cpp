#include <robot_control/arm_lower_to_pitch.h>

void ArmLowerToPitch::init()
{
    armFailed = false;
    armEnded = false;
    initialPitch_ = robotStatus.pitchAngle;
    initialTime_ = ros::Time::now().toSec();
    ROS_WARN("ArmLowerToPitch init()");
    ROS_WARN("initial pitch = %f", initialPitch_);
}

int ArmLowerToPitch::run()
{
    if(((robotStatus.pitchAngle - initialPitch_) <= deltaPitchGoal_) ||
     (robotOutputs.armPosCmd <= -1000) ||
      ((ros::Time::now().toSec() - initialTime_) > timeoutValue_))
    {
        ROS_WARN("ArmLowerToPitch Ended");
        armEnded = true;
        returnValue_ = 1;
    }
    else
    {
        ROS_WARN("ArmLowerToPitch Running");
        robotOutputs.armPosCmd += armPosDelta_;
        if(robotOutputs.armPosCmd < -1000) robotOutputs.armPosCmd = -1000;
        armEnded = false;
        returnValue_ = 0;
    }
    robotOutputs.armStopCmd = 0;
    ROS_INFO("pitch - initialPitch = %f", robotStatus.pitchAngle - initialPitch_);
    ROS_INFO("armPosCmd = %i", robotOutputs.armPosCmd);
    return returnValue_;
}

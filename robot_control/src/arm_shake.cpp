#include <robot_control/arm_shake.h>

void ArmShake::init()
{
    armFailed = false;
    armEnded = false;
    numShakesPerformed_ = 0;
    shakePrevTime_ = 0.0;
    armAtOffsetPosition_ = false;
}

int ArmShake::run()
{
    currentTime_ = ros::Time::now().toSec();
    if(currentTime_ - shakePrevTime_ > shakePeriod_)
    {
        if(armAtOffsetPosition_)
        {
            robotOutputs.armPosCmd = ARM_RAISED;
            armAtOffsetPosition_ = false;
        }
        else
        {
            robotOutputs.armPosCmd = ARM_RAISED - shakePosOffset_;
            armAtOffsetPosition_ = true;
        }
        shakePrevTime_ = currentTime_;
        numShakesPerformed_++;
    }
    if(numShakesPerformed_ >= numShakesToPerform_)
    {
        armEnded = true;
        return 1;
    }
    else
    {
        armEnded = false;
        return 0;
    }
}

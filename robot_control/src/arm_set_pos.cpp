#include <robot_control/arm_set_pos.h>

void ArmSetPos::init()
{
    armPos_ = params.int1;
    armFailed = false;
    armEnded = false;
}

int ArmSetPos::run()
{
    robotOutputs.armPosCmd = armPos_;
    robotOutputs.armStopCmd = 0;
    if(abs(robotStatus.armPos - armPos_) <= armTol) {returnValue_ = 1; armEnded = true;}
    else {returnValue_ = 0; armEnded = false;}
    return returnValue_;
}

#include <robot_control/scoop_set_pos.h>

void ScoopSetPos::init()
{
    scoopPos_ = params.int1;
    scoopFailed = false;
    scoopEnded = false;
}

int ScoopSetPos::run()
{
    robotOutputs.scoopPosCmd = scoopPos_;
    robotOutputs.armStopCmd = 0;
    if(abs(robotStatus.scoopPos - scoopPos_) <= scoopTol) {returnValue_ = 1; scoopEnded = true;}
    else {returnValue_ = 0; scoopEnded = false;}
    return returnValue_;
}

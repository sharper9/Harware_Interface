#include <robot_control/raise_arm.h>

void RaiseArm::init()
{
    scoopFailed = false;
    armFailed = false;
    bucketFailed = false;
    clearDeques();
    pushTask(_armSetPos_);
    armDeque.back()->params.int1 = ARM_RAISED;
}

int RaiseArm::run()
{
    return runDeques();
}

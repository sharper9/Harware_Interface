#include <robot_control/partially_raise_arm.h>

void PartiallyRaiseArm::init()
{
    scoopFailed = false;
    armFailed = false;
    bucketFailed = false;
    clearDeques();
    pushTask(_armSetPos_);
    armDeque.back()->params.int1 = ARM_PARTIALLY_RAISED;
}

int PartiallyRaiseArm::run()
{
    return runDeques();
}

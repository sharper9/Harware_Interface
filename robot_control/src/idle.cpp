#include <robot_control/idle.h>

void Idle::init()
{
    clearDeques();
    driveHalt.init();
    scoopFailed = false;
    armFailed = false;
    bucketFailed = false;
}

int Idle::run()
{
    driveHalt.run();
    scoopHalt.run();
    armHalt.run();
    bucketHalt.run();
    if(driveDeque.empty()) driveDequeEmptyPrev = 1;
    if(scoopDeque.empty()) scoopDequeEmptyPrev = 1;
    if(armDeque.empty()) armDequeEmptyPrev = 1;
    if(bucketDeque.empty()) bucketDequeEmptyPrev = 1;
}

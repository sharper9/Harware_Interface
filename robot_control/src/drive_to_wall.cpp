#include <robot_control/drive_to_wall.h>

void DriveToWall::init()
{
    scoopFailed = false;
    armFailed = false;
    bucketFailed = false;
    pushedToFront_ = params.bool2;
    clearDeques();
    pushTask(_driveUntilLimit_);
}

int DriveToWall::run()
{
    return runDeques();
}

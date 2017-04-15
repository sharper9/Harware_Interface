#include <robot_control/dig.h>

void Dig::init()
{
    scoopFailed = false;
    armFailed = false;
    bucketFailed = false;
    clearDeques();
}

int Dig::run()
{
    return 1; // Temporary until we figure this action out
}

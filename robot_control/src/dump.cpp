#include <robot_control/dump.h>

void Dump::init()
{
    scoopFailed = false;
    armFailed = false;
    bucketFailed = false;
    clearDeques();
}

int Dump::run()
{
    return 1; // Temporary until we figure this action out
}

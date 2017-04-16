#include <robot_control/bucket_halt.h>

void BucketHalt::init()
{
    robotOutputs.bucketStopCmd = 1;
}

int BucketHalt::run()
{
    robotOutputs.bucketStopCmd = 1;
	return 1;
}

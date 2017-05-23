#include <robot_control/lower_bucket.h>

void LowerBucket::init()
{
    scoopFailed = false;
    armFailed = false;
    bucketFailed = false;
    clearDeques();
    pushTask(_bucketSetPos_);
    bucketDeque.back()->params.int1 = BUCKET_LOWERED;
}

int LowerBucket::run()
{
    return runDeques();
}

#include <robot_control/partially_raise_bucket.h>

void PartiallyRaiseBucket::init()
{
    scoopFailed = false;
    armFailed = false;
    bucketFailed = false;
    clearDeques();
    pushTask(_bucketSetPos_);
    bucketDeque.back()->params.int1 = BUCKET_PARTIALLY_RAISED;
}

int PartiallyRaiseBucket::run()
{
    return runDeques();
}

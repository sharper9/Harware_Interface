#include <robot_control/raise_bucket.h>

void RaiseBucket::init()
{
    scoopFailed = false;
    armFailed = false;
    bucketFailed = false;
    clearDeques();
    pushTask(_bucketSetPos_);
    bucketDeque.back()->params.int1 = BUCKET_RAISED;
}

int RaiseBucket::run()
{
    return runDeques();
}

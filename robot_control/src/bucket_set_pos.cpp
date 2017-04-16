#include <robot_control/bucket_set_pos.h>

void BucketSetPos::init()
{
    bucketPos_ = params.int1;
    bucketFailed = false;
    bucketEnded = false;
}

int BucketSetPos::run()
{
    robotOutputs.bucketPosCmd = bucketPos_;
    robotOutputs.armStopCmd = 0;
    if(abs(robotStatus.bucketPos - bucketPos_) <= bucketTol) {returnValue_ = 1; bucketEnded = true;}
    else {returnValue_ = 0; bucketEnded = false;}
    return returnValue_;
}

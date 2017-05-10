#include <robot_control/bucket_shake.h>

void BucketShake::init()
{
    bucketFailed = false;
    bucketEnded = false;
    numShakesPerformed_ = 0;
    shakePrevTime_ = 0.0;
    bucketAtOffsetPosition_ = false;
}

int BucketShake::run()
{
    currentTime_ = ros::Time::now().toSec();
    if(currentTime_ - shakePrevTime_ > shakePeriod_)
    {
        if(bucketAtOffsetPosition_) robotOutputs.bucketPosCmd = BUCKET_RAISED;
        else robotOutputs.bucketPosCmd = BUCKET_RAISED - shakePosOffset_;
        shakePrevTime_ = currentTime_;
        numShakesPerformed_++;
    }
    if(numShakesPerformed_ >= numShakesToPerform_)
    {
        bucketEnded = true;
        return 1;
    }
    else
    {
        bucketEnded = false;
        return 0;
    }
}

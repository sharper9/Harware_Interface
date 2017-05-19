#include <robot_control/prepare_arm_bucket.h>

void PrepareArmBucket::init()
{
    scoopFailed = false;
    armFailed = false;
    bucketFailed = false;
    step_ = _moveArm;
    completed_ = false;
    clearDeques();
    pushTask(_armSetPos_);
    armDeque.back()->params.int1 = ARM_PRE_LOWER;
}

int PrepareArmBucket::run()
{
    switch(step_)
    {
    case _moveArm:
        if(runDeques())
        {
            step_ = _moveBucket;
            pushTask(_bucketSetPos_);
            bucketDeque.back()->params.int1 = BUCKET_PRE_RAISED;
        }
        else step_ = _moveArm;
        completed_ = false;
        break;
    case _moveBucket:
        if(runDeques()) completed_ = true;
        else
        {
            step_ = _moveBucket;
            completed_ = false;
        }
        break;
    }
    return completed_;
}

#ifndef BUCKET_SHAKE_H
#define BUCKET_SHAKE_H
#include "task.h"

class BucketShake : public Task
{
public:
    void init();
    int run();
private:
    int numShakesPerformed_;
    double currentTime_;
    double shakePrevTime_;
    bool bucketAtOffsetPosition_;
    const int numShakesToPerform_ = 5;
    const int shakePosOffset_ = 150;
    const double shakePeriod_ = 0.75; // sec
};

#endif // BUCKET_SHAKE_H

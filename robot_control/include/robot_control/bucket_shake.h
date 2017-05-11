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
    const int numShakesToPerform_ = 3;
    const int shakePosOffset_ = 5;
    const double shakePeriod_ = 1.0; // sec
};

#endif // BUCKET_SHAKE_H

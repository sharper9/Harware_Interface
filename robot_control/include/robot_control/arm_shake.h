#ifndef ARM_SHAKE_H
#define ARM_SHAKE_H
#include "task.h"

class ArmShake : public Task
{
public:
    void init();
    int run();
private:
    int numShakesPerformed_;
    double currentTime_;
    double shakePrevTime_;
    bool armAtOffsetPosition_;
    const int numShakesToPerform_ = 10;
    const int shakePosOffset_ = 100;
    const double shakePeriod_ = 0.5; // sec
};

#endif // ARM_SHAKE_H

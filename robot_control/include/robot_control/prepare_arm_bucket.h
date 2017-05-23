#ifndef PREPARE_ARM_BUCKET_H
#define PREPARE_ARM_BUCKET_H
#include "action.h"

class PrepareArmBucket : public Action
{
public:
    void init();
    int run();
private:
    enum PREPARE_ARM_BUCKET_STEP_T {_moveArm, _moveBucket} step_;
    bool completed_;
};

#endif // PREPARE_ARM_BUCKET_H

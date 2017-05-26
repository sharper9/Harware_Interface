#ifndef DIG_H
#define DIG_H
#include "action.h"

class Dig : public Action
{
public:
    void init();
    int run();
private:
    enum DIG_STEP_T {_lowerScoop, _lowerArm, _drive1, _midScoop, _drive2, _raiseScoop, _drive3, _raiseArm} step_;
    bool digCompleted_;
    bool taskPushed_;
    bool taskFinished_;
    TASK_TYPE_T taskToPush_;
    enum TYPE_OF_TASK_PUSHED_T {__drive, __scoop, __arm, __bucket} typeOfTaskPushed_;
    float valueToPush_;
    float digPitchAngle_;
    const float digDriveDistance_ = 0.5; // m
    const float forwardAndBackUpDistance_ = 0.4; // m
};

#endif // DIG_H

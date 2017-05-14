#ifndef DIG_H
#define DIG_H
#include "action.h"

class Dig : public Action
{
public:
    void init();
    int run();
private:
    enum DIG_STEP_T {_lowerArm, _lowerScoop, _drive1, _raiseScoop, _drive2, _raiseArm, _shake} step_;
    bool digCompleted_;
    bool taskPushed_;
    bool taskFinished_;
    TASK_TYPE_T taskToPush_;
    enum TYPE_OF_TASK_PUSHED_T {__drive, __scoop, __arm, __bucket} typeOfTaskPushed_;
    float valueToPush_;
    const float digDriveDistance = 0.3; // m
};

#endif // DIG_H

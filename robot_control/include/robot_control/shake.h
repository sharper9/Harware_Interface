#ifndef SHAKE_H
#define SHAKE_H
#include "action.h"

class Shake : public Action
{
public:
    void init();
    int run();
private:
    enum SHAKE_STEP_T {_driveForward, _backUp} step_;
    bool shakeCompleted_;
    bool taskPushed_;
    bool taskFinished_;
    TASK_TYPE_T taskToPush_;
    enum TYPE_OF_TASK_PUSHED_T {__drive, __scoop, __arm, __bucket} typeOfTaskPushed_;
    float valueToPush_;
    const float forwardAndBackUpDistance_ = 0.4; // m
};

#endif // SHAKE_H

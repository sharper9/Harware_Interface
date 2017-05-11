#ifndef DUMP_H
#define DUMP_H
#include "action.h"

class Dump : public Action
{
public:
    void init();
    int run();
private:
    enum DUMP_STEP_T {_moveArm, _raiseBucket, _shake, _lowerBucket, _returnArm} step_;
    bool dumpCompleted_;
    bool taskPushed_;
    bool taskFinished_;
    TASK_TYPE_T taskToPush_;
    enum TYPE_OF_TASK_PUSHED_T {__drive, __scoop, __arm, __bucket} typeOfTaskPushed_;
    float valueToPush_;
};

#endif // DUMP_H

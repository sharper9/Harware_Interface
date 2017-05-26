#ifndef SHAKE_H
#define SHAKE_H
#include "action.h"

class Shake : public Action
{
public:
    void init();
    int run();
private:
    enum SHAKE_STEP_T {_driveForward, _backUp, _finalBackUp} step_;
    bool shakeCompleted_;
    bool taskPushed_;
    bool taskFinished_;
    TASK_TYPE_T taskToPush_;
    enum TYPE_OF_TASK_PUSHED_T {__drive, __scoop, __arm, __bucket} typeOfTaskPushed_;
    float valueToPush_;
    const float forwardDistance_ = 0.4; // m
    const float backupDistance_ = 0.3; // 
    const float finalBackupDistance_ = 0.5; // m
};

#endif // SHAKE_H

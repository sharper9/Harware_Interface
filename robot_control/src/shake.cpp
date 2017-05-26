#include <robot_control/shake.h>

void Shake::init()
{
    scoopFailed = false;
    armFailed = false;
    bucketFailed = false;
    step_ = _driveForward;
    shakeCompleted_ = false;
    taskPushed_ = false;
    taskFinished_ = false;
    clearDeques();
    pushTask(_scoopSetPos_);
    scoopDeque.back()->params.int1 = SCOOP_RAISED;
    pushTask(_armSetPos_);
    armDeque.back()->params.int1 = ARM_RAISED;
}

int Shake::run()
{
    if(taskPushed_)
    {
        taskFinished_ = runDeques();
        if(taskFinished_ && !shakeCompleted_)
        {
            taskPushed_ = false;
        }
    }
    else
    {
        switch(step_)
        {
        case _driveForward:
            taskToPush_ = _driveStraight_;
            typeOfTaskPushed_ = __drive;
            valueToPush_ = forwardDistance_;
            pushTask(_armShake_);
            step_ = _backUp;
            break;
        case _backUp:
            taskToPush_ = _driveStraight_;
            typeOfTaskPushed_ = __drive;
            valueToPush_ = -backupDistance_;
            pushTask(_armShake_);
            step_ = _finalBackUp;
            break;
        case _finalBackUp:
            taskToPush_ = _driveStraight_;
            typeOfTaskPushed_ = __drive;
            valueToPush_ = -finalBackupDistance_;
            shakeCompleted_ = true;
            break;
        }
        pushTask(taskToPush_);
        switch(typeOfTaskPushed_)
        {
        case __drive:
            driveDeque.back()->params.float1 = valueToPush_;
            break;
        case __scoop:
            scoopDeque.back()->params.int1 = (int)valueToPush_;
            break;
        case __arm:
            armDeque.back()->params.int1 = (int)valueToPush_;
            break;
        case __bucket:
            bucketDeque.back()->params.int1 = (int)valueToPush_;
            break;
        }
        taskFinished_ = false;
        taskPushed_ = true;
    }
    return (shakeCompleted_ && taskFinished_);
}

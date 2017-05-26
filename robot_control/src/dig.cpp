#include <robot_control/dig.h>

void Dig::init()
{
    scoopFailed = false;
    armFailed = false;
    bucketFailed = false;
    step_ = _lowerScoop;
    digCompleted_ = false;
    taskPushed_ = false;
    taskFinished_ = false;
    digPitchAngle_ = params.float1;
    clearDeques();
}

int Dig::run()
{
    if(taskPushed_)
    {
        taskFinished_ = runDeques();
        if(taskFinished_ && !digCompleted_)
        {
            taskPushed_ = false;
        }
    }
    else
    {
        switch(step_)
        {
        case _lowerScoop:
            taskToPush_ = _scoopSetPos_;
            typeOfTaskPushed_ = __scoop;
            valueToPush_ = SCOOP_LOWERED;
            pushTask(_armSetPos_);
            armDeque.back()->params.int1 = ARM_LOWERED;
            step_ = _lowerArm;
            break;
        case _lowerArm:
            taskToPush_ = _armLowerToPitch_;
            typeOfTaskPushed_ = __arm;
            valueToPush_ = (int)digPitchAngle_;
            step_ = _drive1;
            break;
        case _drive1:
            taskToPush_ = _driveStraight_;
            typeOfTaskPushed_ = __drive;
            valueToPush_ = digDriveDistance_;
            step_ = _midScoop;
            break;
        case _midScoop:
            taskToPush_ = _scoopSetPos_;
            typeOfTaskPushed_ = __scoop;
            valueToPush_ = SCOOP_MID;
            step_ = _drive2;
            break;
        case _drive2:
            taskToPush_ = _driveStraight_;
            typeOfTaskPushed_ = __drive;
            valueToPush_ = digDriveDistance_;
            step_ = _raiseScoop;
            break;
        case _raiseScoop:
            taskToPush_ = _scoopSetPos_;
            typeOfTaskPushed_ = __scoop;
            valueToPush_ = SCOOP_RAISED;
            step_ = _drive3;
            break;
        case _drive3:
            taskToPush_ = _driveStraight_;
            typeOfTaskPushed_ = __drive;
            valueToPush_ = digDriveDistance_;
            step_ = _raiseArm;
            break;
        case _raiseArm:
            taskToPush_ = _armSetPos_;
            typeOfTaskPushed_ = __arm;
            valueToPush_ = ARM_RAISED;
            digCompleted_ = true;
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
    return (digCompleted_ && taskFinished_);
}

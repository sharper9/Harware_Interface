#include <robot_control/dig.h>

void Dig::init()
{
    scoopFailed = false;
    armFailed = false;
    bucketFailed = false;
    step_ = _lowerArm;
    digCompleted_ = false;
    taskPushed_ = false;
    taskFinished_ = false;
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
        ROS_INFO("step = %i",step_);
        switch(step_)
        {
        case _lowerArm:
            taskToPush_ = _armSetPos_;
            typeOfTaskPushed_ = __arm;
            valueToPush_ = ARM_LOWERED;
            step_ = _lowerScoop;
            break;
        case _lowerScoop:
            taskToPush_ = _scoopSetPos_;
            typeOfTaskPushed_ = __scoop;
            valueToPush_ = SCOOP_LOWERED;
            step_ = _drive1;
            break;
        case _drive1:
            taskToPush_ = _driveStraight_;
            typeOfTaskPushed_ = __drive;
            valueToPush_ = digDriveDistance;
            step_ = _raiseScoop;
            break;
        case _raiseScoop:
            taskToPush_ = _scoopSetPos_;
            typeOfTaskPushed_ = __scoop;
            valueToPush_ = SCOOP_RAISED;
            step_ = _drive2;
            break;
        case _drive2:
            taskToPush_ = _driveStraight_;
            typeOfTaskPushed_ = __drive;
            valueToPush_ = digDriveDistance;
            step_ = _raiseArm;
            break;
        case _raiseArm:
            taskToPush_ = _armSetPos_;
            typeOfTaskPushed_ = __arm;
            valueToPush_ = ARM_RAISED;
            step_ = _backUp;
            break;
        case _backUp:
            taskToPush_ = _driveStraight_;
            typeOfTaskPushed_ = __drive;
            valueToPush_ = -backUpDistance;
            step_ = _shake;
            break;
        case _shake:
            taskToPush_ = _armShake_;
            typeOfTaskPushed_ = __arm;
            valueToPush_ = 0;
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

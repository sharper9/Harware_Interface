#include <robot_control/dump.h>

void Dump::init()
{
    scoopFailed = false;
    armFailed = false;
    bucketFailed = false;
    step_ = _moveArm;
    dumpCompleted_ = false;
    taskPushed_ = false;
    taskFinished_ = false;
    clearDeques();
}

int Dump::run()
{
    if(taskPushed_)
    {
        if(typeOfTaskPushed_ == __none)
        {
            if((ros::Time::now().toSec() - waitStartTime_) > waitTime_) taskFinished_ = true;
            else taskFinished_ = false;
        }
        else taskFinished_ = runDeques();
        if(taskFinished_ && !dumpCompleted_)
        {
            taskPushed_ = false;
        }
    }
    else
    {
        switch(step_)
        {
        case _moveArm:
            taskToPush_ = _armSetPos_;
            typeOfTaskPushed_ = __arm;
            valueToPush_ = ARM_DUMP;
            step_ = _raiseBucket;
            break;
        case _raiseBucket:
            taskToPush_ = _bucketSetPos_;
            typeOfTaskPushed_ = __bucket;
            valueToPush_ = BUCKET_RAISED;
            step_ = _waitForSand;
            break;
        case _waitForSand:
            waitStartTime_ = ros::Time::now().toSec();
            typeOfTaskPushed_ = __none;
            step_ = _forward1;
            break;
        case _forward1:
            taskToPush_ = _driveStraight_;
            typeOfTaskPushed_ = __drive;
            valueToPush_ = driveForwardDistance_;
            step_ = _back1;
            break;
        case _back1:
            taskToPush_ = _driveUntilLimit_;
            typeOfTaskPushed_ = __drive;
            valueToPush_ = 0.0;
            step_ = _forward2;
            break;
        case _forward2:
            taskToPush_ = _driveStraight_;
            typeOfTaskPushed_ = __drive;
            valueToPush_ = driveForwardDistance_;
            step_ = _back2;
            break;
        case _back2:
            taskToPush_ = _driveUntilLimit_;
            typeOfTaskPushed_ = __drive;
            valueToPush_ = 0.0;
            step_ = _lowerBucket;
        case _lowerBucket:
            taskToPush_ = _bucketSetPos_;
            typeOfTaskPushed_ = __bucket;
            valueToPush_ = BUCKET_LOWERED;
            step_ = _returnArm;
            break;
        case _returnArm:
            taskToPush_ = _armSetPos_;
            typeOfTaskPushed_ = __arm ;
            valueToPush_ = ARM_RAISED;
            dumpCompleted_ = true;
            break;
        }
        if(typeOfTaskPushed_ != __none)
        {
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
        }
        taskFinished_ = false;
        taskPushed_ = true;
    }
    return (dumpCompleted_ && taskFinished_);
}

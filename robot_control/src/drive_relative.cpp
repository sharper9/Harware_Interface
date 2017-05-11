#include <robot_control/drive_relative.h>

void DriveRelative::init()
{
    scoopFailed = false;
    armFailed = false;
    bucketFailed = false;
    desiredX_ = params.float1*cos(DEG2RAD*(params.float2 + robotStatus.heading)) + robotStatus.xPos;
    desiredY_ = params.float1*sin(DEG2RAD*(params.float2 + robotStatus.heading)) + robotStatus.yPos;
    desiredEndHeading_ = params.float3;
    endHeading_ = params.bool1;
    pushedToFront_ = params.bool2;
    driveBackwards_ = params.bool3;
    nextGlobalX = desiredX_;
    nextGlobalY = desiredY_;
    clearDeques();
    step_ = _computeManeuver;
    //if(pushedToFront_) initDequesFront(); // *** This might have been important...***
}

int DriveRelative::run()
{
    switch(step_)
    {
    case _computeManeuver:
        calculatePath_();
        if(endHeading_)
        {
            pushTask(_pivot_);
            driveDeque.back()->params.float1 = angleToTurn_;
            pushTask(_driveStraight_);
            driveDeque.back()->params.float1 = distanceToDrive_;
            pushTask(_pivot_);
            uXCurrentHeading_ = cos(DEG2RAD*(robotStatus.heading+angleToTurn_));
            uYCurrentHeading_ = sin(DEG2RAD*(robotStatus.heading+angleToTurn_));
            uXDesiredEndHeading_ = cos(DEG2RAD*desiredEndHeading_);
            uYDesiredEndHeading_ = sin(DEG2RAD*desiredEndHeading_);
            if(asin(uXCurrentHeading_*uYDesiredEndHeading_ - uXDesiredEndHeading_*uYCurrentHeading_)>=0.0) signDesiredEndHeading_ = 1.0;
            else signDesiredEndHeading_ = -1.0;
            driveDeque.back()->params.float1 = RAD2DEG*signDesiredEndHeading_*acos(uXCurrentHeading_*uXDesiredEndHeading_ + uYCurrentHeading_*uYDesiredEndHeading_);
            ROS_WARN("current heading = %f, desired heading  = %f, angle to turn = %f", fmod(robotStatus.heading, 360.0), desiredEndHeading_, driveDeque.back()->params.float1);
            driveCompleted_ = false;
        }
        else
        {
            pushTask(_pivot_);
            driveDeque.back()->params.float1 = angleToTurn_;
            pushTask(_driveStraight_);
            driveDeque.back()->params.float1 = distanceToDrive_;
            driveCompleted_ = false;
        }
        step_ = _performManeuver;
        break;
    case _performManeuver:
        driveCompleted_ = runDeques();
        if(driveCompleted_) step_ = _computeManeuver;
        else step_ = _performManeuver;
        break;
    }
    return driveCompleted_;
}

void DriveRelative::calculatePath_()
{
    xErr_ = desiredX_-robotStatus.xPos;
    yErr_ = desiredY_-robotStatus.yPos;
    uXDes_ = xErr_/hypot(xErr_,yErr_);
    uYDes_ = yErr_/hypot(xErr_,yErr_);
    if(driveBackwards_)
    {
        uXAct_ = -cos(robotStatus.heading*PI/180.0);
        uYAct_ = -sin(robotStatus.heading*PI/180.0);
        distanceToDrive_ = -hypot(xErr_,yErr_);
    }
    else
    {
        uXAct_ = cos(robotStatus.heading*PI/180.0);
        uYAct_ = sin(robotStatus.heading*PI/180.0);
        distanceToDrive_ = hypot(xErr_,yErr_);
    }
    if(asin(uXAct_*uYDes_-uXDes_*uYAct_)>=0) newHeadingSign_ = 1.0;
    else newHeadingSign_ = -1.0;
    angleToTurn_ = (180.0/PI)*(newHeadingSign_)*acos(uXAct_*uXDes_+uYAct_*uYDes_);
}

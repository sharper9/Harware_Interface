#include <robot_control/drive_global.h>

void DriveGlobal::init()
{
    scoopFailed = false;
    armFailed = false;
    bucketFailed = false;
	desiredX_ = params.float1;
	desiredY_ = params.float2;
    desiredEndHeading_ = params.float3;
	endHeading_ = params.bool1;
    pushedToFront_ = params.bool2;
    driveBackwards_ = params.bool3;
    nextGlobalX = params.float1;
    nextGlobalY = params.float2;
    clearDeques();
    step_ = _computeManeuver;
    //if(pushedToFront_) initDequesFront(); // *** This might have been important...***
}

int DriveGlobal::run()
{
    switch(step_)
    {
    case _computeManeuver:
        calculatePath_();
        if(distanceToDrive_<finalPositionDistanceTolerance)
        {
            if(endHeading_ && (fabs(robotStatus.heading - desiredEndHeading_) > finalHeadingAngleTolerance))
            {
                pushTask(_pivot_);
                candidateEndHeadingAngleToTurn_[0] = desiredEndHeading_ - fmod(robotStatus.heading, 360.0);
                candidateEndHeadingAngleToTurn_[1] = -desiredEndHeading_ - fmod(robotStatus.heading, 360.0);
                if(fabs(candidateEndHeadingAngleToTurn_[0]) < fabs(candidateEndHeadingAngleToTurn_[1]))
                    driveDeque.back()->params.float1 = candidateEndHeadingAngleToTurn_[0];
                else
                    driveDeque.back()->params.float1 = candidateEndHeadingAngleToTurn_[1];
                driveCompleted_ = false;
                step_ = _performManeuver;
            }
            else
            {
                driveCompleted_ = true;
                step_ = _computeManeuver;
            }
        }
        else
        {
            pushTask(_pivot_);
            driveDeque.back()->params.float1 = angleToTurn_;
            pushTask(_driveStraight_);
            driveDeque.back()->params.float1 = distanceToDrive_;
            driveCompleted_ = false;
            step_ = _performManeuver;
        }
        break;
    case _performManeuver:
        if(runDeques()) step_ = _computeManeuver;
        else step_ = _performManeuver;
        break;
    }
    return driveCompleted_;
}

void DriveGlobal::calculatePath_()
{
	xErr_ = desiredX_-robotStatus.xPos;
	yErr_ = desiredY_-robotStatus.yPos;
	uXDes_ = xErr_/hypot(xErr_,yErr_);
	uYDes_ = yErr_/hypot(xErr_,yErr_);
	uXAct_ = cos(robotStatus.heading*PI/180.0);
	uYAct_ = sin(robotStatus.heading*PI/180.0);
	if(asin(uXAct_*uYDes_-uXDes_*uYAct_)>=0) newHeadingSign_ = 1.0;
	else newHeadingSign_ = -1.0;
    if(driveBackwards_)
    {
        angleToTurn_ = (180.0/PI)*(newHeadingSign_)*acos(uXAct_*uXDes_+uYAct_*uYDes_)+180.0;
        distanceToDrive_ = -hypot(xErr_,yErr_);
    }
    else
    {
        angleToTurn_ = (180.0/PI)*(newHeadingSign_)*acos(uXAct_*uXDes_+uYAct_*uYDes_);
        distanceToDrive_ = hypot(xErr_,yErr_);
    }
    if(distanceToDrive_>incrementalDriveDistance) distanceToDrive_ = incrementalDriveDistance;
}

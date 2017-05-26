#include <robot_control/close_to_wall.h>

bool CloseToWall::runProc()
{
    switch(state)
    {
    case _init_:
        procsBeingExecuted[procType] = true;
        procsToExecute[procType] = false;
        procsToResume[procType] = false;
        computeDriveSpeeds();
        sendPause();
        performFullPoseUpdate = true;
        state = _exec_;
        stage = _fullPose;
        resetQueueEmptyCondition();
        break;
    case _exec_:
        procsBeingExecuted[procType] = true;
        procsToExecute[procType] = false;
        procsToResume[procType] = false;
        computeDriveSpeeds();
        switch(stage)
        {
        case _fullPose:
            if(robotStatus.fullPoseFound || queueEmptyTimedOut)
            {
                sendUnPause();
                numWaypointsToTravel = 1;
                clearAndResizeWTT();
                if(bucketFull)
                {
                    if(hypot(depositWaypointX - robotStatus.xPos, depositWaypointY - robotStatus.yPos) < depositWaypointDistanceTolerance &&
                            fabs(robotStatus.heading) < depositWaypointAngleTolerance)
                    {
                        state = _finish_;
                    }
                    else
                    {
                        waypointsToTravel.at(0).x = depositWaypointX;
                        waypointsToTravel.at(0).y = depositWaypointY;
                        if(hypot(depositWaypointX - robotStatus.xPos, depositWaypointX - robotStatus.yPos) < depositWaypointDistanceTolerance)
                        {
                            sendDriveRel(0.0, 0.0, true, 0.0, true, false);
                        }
                        else
                        {
                            sendDriveGlobal(true, true, 0.0, true);

                        }
                        stage = _driveToLocation;
                        state = _exec_;
                    }
                }
                else
                {
                    if(hypot(miningRegionTargetXDistance - robotStatus.xPos, 0.0 - robotStatus.yPos) < digWaypointDistanceTolerance &&
                            fabs(robotStatus.heading) < digWaypointAngleTolerance && robotStatus.xPos > miningRegionMinXDistance)
                    {
                        state = _finish_;
                    }
                    else
                    {
                        waypointsToTravel.at(0).x = miningRegionTargetXDistance;
                        waypointsToTravel.at(0).y = 0.0;
                        if(hypot(miningRegionTargetXDistance - robotStatus.xPos, 0.0 - robotStatus.yPos) < digWaypointDistanceTolerance)
                        {
                            sendDriveRel(0.0, 0.0, true, 0.0, true, false);
                        }
                        else
                        {
                            if(robotStatus.xPos >= miningRegionTargetXDistance)
                            {
                                sendDriveGlobal(true, true, 0.0, true);
                            }
                            else
                            {
                                sendDriveGlobal(true, true, 0.0, false);
                            }
                        }
                        stage = _driveToLocation;
                        state = _exec_;
                    }
                }
            }
            else
            {
                stage = _fullPose;
                state = _exec_;
            }
            break;
        case _driveToLocation:
            if((execLastProcType == procType && execLastSerialNum == serialNum) || queueEmptyTimedOut)
            {
                resetQueueEmptyCondition();
                performFullPoseUpdate = true;
                sendPause();
                stage = _fullPose;
            }
            else
            {
                stage = _driveToLocation;
            }
            state = _exec_;
            break;
        }
        serviceQueueEmptyCondition();
        break;
    case _interrupt_:
        procsBeingExecuted[procType] = false;
        procsToInterrupt[procType] = false;
        sendDequeClearAll();
        state = _init_;
        break;
    case _finish_:
        tooCloseToWall = false;
        tooCloseToWallLockout = false;
        prevXPos = robotStatus.xPos;
        prevYPos = robotStatus.yPos;
        prevPosUnchangedTime = ros::Time::now().toSec();
        procsBeingExecuted[procType] = false;
        procsToExecute[procType] = false;
        procsToResume[procType] = false;
        state = _init_;
        break;
    }
}

#include <robot_control/drive_to_dig.h>

bool DriveToDig::runProc()
{
    switch(state)
    {
    case _init_:
        procsBeingExecuted[procType] = true;
        procsToExecute[procType] = false;
        procsToResume[procType] = false;
        computeDriveSpeeds();
        numWaypointsToTravel = 1;
        ROS_WARN("dig planning map, upper = %f, lower = %f", digPlanningMap.atPos(robotStatus.xPos,robotStatus.yPos+mapYOffset).headingUpperLimit, digPlanningMap.atPos(robotStatus.xPos,robotStatus.yPos+mapYOffset).headingLowerLimit);
        chosenHeading = (static_cast<float>(rand()) / static_cast<float>(RAND_MAX))*
                (digPlanningMap.atPos(robotStatus.xPos,robotStatus.yPos+mapYOffset).headingUpperLimit -
                 digPlanningMap.atPos(robotStatus.xPos,robotStatus.yPos+mapYOffset).headingLowerLimit) + digPlanningMap.atPos(robotStatus.xPos,robotStatus.yPos+mapYOffset).headingLowerLimit;
        distanceToDrive = (miningRegionTargetXDistance - robotStatus.xPos)/cos(DEG2RAD*chosenHeading);
        angleToTurn = chosenHeading - robotStatus.heading;
        sendDriveRel(distanceToDrive, angleToTurn, true, 0.0, false, false);
        //sendWait(5.0, false); // TODO: remove, this is temporary for testing
        state = _exec_;
        resetQueueEmptyCondition();
        break;
    case _exec_:
        procsBeingExecuted[procType] = true;
        procsToExecute[procType] = false;
        procsToResume[procType] = false;
        computeDriveSpeeds();
        if(execLastProcType == procType && execLastSerialNum == serialNum) performFullPoseUpdate = true;
        if((execLastProcType == procType && execLastSerialNum == serialNum && robotStatus.fullPoseFound) || queueEmptyTimedOut) state = _finish_;
        else state = _exec_;
        serviceQueueEmptyCondition();
        break;
    case _interrupt_:
        procsBeingExecuted[procType] = false;
        procsToInterrupt[procType] = false;
        sendDequeClearAll();
        state = _init_;
        break;
    case _finish_:
        if(robotStatus.xPos > miningRegionMinXDistance) atMineLocation = true; // TODO: need to wait for ranging radio update before reevaluating this
        else atMineLocation = false;
        procsBeingExecuted[procType] = false;
        procsToExecute[procType] = false;
        procsToResume[procType] = false;
        state = _init_;
        break;
    }
}

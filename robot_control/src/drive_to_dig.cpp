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
        chosenHeading = (static_cast<float>(rand()) / static_cast<float>(RAND_MAX))*
                (digPlanningMap.atPos(robotStatus.xPos,robotStatus.yPos).headingUpperLimit -
                 digPlanningMap.atPos(robotStatus.xPos,robotStatus.yPos).headingLowerLimit) + digPlanningMap.atPos(robotStatus.xPos,robotStatus.yPos).headingLowerLimit;
        distanceToDrive = (miningRegionTargetXDistance - robotStatus.xPos)/cos(DEG2RAD*chosenHeading);
        angleToTurn = chosenHeading - robotStatus.heading;
        sendDriveRel(distanceToDrive, angleToTurn, false, 0.0, false, false);
        state = _exec_;
        resetQueueEmptyCondition();
        break;
    case _exec_:
        procsBeingExecuted[procType] = true;
        procsToExecute[procType] = false;
        procsToResume[procType] = false;
        computeDriveSpeeds();
        if((execLastProcType == procType && execLastSerialNum == serialNum) || queueEmptyTimedOut) state = _finish_;
        else state = _exec_;
        serviceQueueEmptyCondition();
        break;
    case _interrupt_:
        procsBeingExecuted[procType] = false;
        procsToInterrupt[procType] = false;
        state = _exec_;
        break;
    case _finish_:
        atMineLocation = true;
        procsBeingExecuted[procType] = false;
        procsToExecute[procType] = false;
        procsToResume[procType] = false;
        state = _init_;
        break;
    }
}

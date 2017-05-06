#include <robot_control/drive_to_deposit.h>

bool DriveToDeposit::runProc()
{
    switch(state)
    {
    case _init_:
        procsBeingExecuted[procType] = true;
        procsToExecute[procType] = false;
        procsToResume[procType] = false;
        computeDriveSpeeds();
        numWaypointsToTravel = 1;
        clearAndResizeWTT();
        waypointsToTravel.at(0).x = depositWaypointX;
        waypointsToTravel.at(0).y = depositWaypointY;
        sendDriveGlobal(false, false, 0.0, false);
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
        atDepositLocation = true;
        if((hypot(robotStatus.xPos - depositWaypointX, robotStatus.yPos - depositWaypointY) < depositWaypointDistanceTolerance) &&
                (fabs(fmod(robotStatus.heading + 180.0, 360.0) - 180.0) < depositWaypointDistanceTolerance))
            confirmedAtDepositLocation = true;
        else confirmedAtDepositLocation = true; // TODO: should be false, but need to implement realign procedure first
        procsBeingExecuted[procType] = false;
        procsToExecute[procType] = false;
        procsToResume[procType] = false;
        state = _init_;
        break;
    }
    return true;
}

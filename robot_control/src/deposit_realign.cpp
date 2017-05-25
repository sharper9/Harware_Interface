#include <robot_control/deposit_realign.h>

bool DepositRealign::runProc()
{
    switch(state)
    {
    case _init_:
        procsBeingExecuted[procType] = true;
        procsToExecute[procType] = false;
        procsToResume[procType] = false;
        depositWaypointDistanceTolerance += depositWaypointDistanceToleranceIncrement;
        if(depositWaypointDistanceTolerance > depositWaypointDistanceToleranceMax) depositWaypointDistanceTolerance = depositWaypointDistanceToleranceMax;
        depositWaypointAngleTolerance += depositWaypointAngleToleranceIncrement;
        if(depositWaypointAngleTolerance > depositWaypointAngleToleranceMax) depositWaypointAngleTolerance = depositWaypointAngleToleranceMax;
        computeDriveSpeeds();
        //sendPrepareArmBucket();
        numWaypointsToTravel = 1;
        clearAndResizeWTT();
        waypointsToTravel.at(0).x = depositWaypointRecoverX;
        waypointsToTravel.at(0).y = depositWaypointRecoverY;
        sendDriveGlobal(false, true, 0.0, false);
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
        atDepositLocation = false;
        confirmedAtDepositLocation = false;
        procsBeingExecuted[procType] = false;
        procsToExecute[procType] = false;
        procsToResume[procType] = false;
        state = _init_;
        break;
    }
    return true;
}

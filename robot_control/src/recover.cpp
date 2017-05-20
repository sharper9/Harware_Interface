#include <robot_control/recover.h>

bool Recover::runProc()
{
    switch(state)
    {
    case _init_:
        procsBeingExecuted[procType] = true;
        procsToExecute[procType] = false;
        procsToResume[procType] = false;
        computeDriveSpeeds();
        sendDriveRel(0.0, 360.0, false, 0.0, true, false);
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
        sendDequeClearAll();
        state = _init_;
        break;
    case _finish_:
        recoverLockout = false;
        stuck = false;
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

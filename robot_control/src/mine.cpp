#include <robot_control/mine.h>

bool Mine::runProc()
{
    switch(state)
    {
    case _init_:
        procsBeingExecuted[procType] = true;
        procsToExecute[procType] = false;
        procsToResume[procType] = false;
        computeDriveSpeeds();
        for(int i=0; i<numDigsPerMine; i++)
        {
            sendDig();
        }
        finalSerialNum = serialNum;
        tooCloseToWallLatch.LE_Latch(0);
        state = _exec_;
        resetQueueEmptyCondition();
        break;
    case _exec_:
        procsBeingExecuted[procType] = true;
        procsToExecute[procType] = false;
        procsToResume[procType] = false;
        computeDriveSpeeds();
        tooCloseToWall = (robotStatus.xPos + robotCenterToScoopLength) >= (DIG_MAP_X_LEN - miningWallBufferDistance);
        if(0 && tooCloseToWallLatch.LE_Latch(tooCloseToWall)) // TODO: remove "0 &&"
        {
            sendDriveRel(backUpDistance, 0.0, false, 0.0, true, true);
        }
        if((execLastProcType == procType && execLastSerialNum == finalSerialNum) || queueEmptyTimedOut) state = _finish_;
        else state = _exec_;
        serviceQueueEmptyCondition();
        break;
    case _interrupt_:
        procsBeingExecuted[procType] = false;
        procsToInterrupt[procType] = false;
        state = _exec_;
        break;
    case _finish_:
        bucketFull = true;
        atMineLocation = false;
        procsBeingExecuted[procType] = false;
        procsToExecute[procType] = false;
        procsToResume[procType] = false;
        state = _init_;
        break;
    }
}

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
        tooCloseToWall = (((robotStatus.xPos + robotCenterToScoopLength*cos(DEG2RAD*robotStatus.heading)) >= (DIG_MAP_X_LEN - miningWallBufferDistance))
                          || ((robotStatus.yPos + robotCenterToScoopLength*sin(DEG2RAD*robotStatus.heading)) >= (DIG_MAP_Y_LEN - miningWallBufferDistance))
                              || ((robotStatus.yPos + robotCenterToScoopLength*sin(DEG2RAD*robotStatus.heading)) <= miningWallBufferDistance));
        if(tooCloseToWallLatch.LE_Latch(tooCloseToWall))
        {
            ROS_WARN("Scoop too close to wall. Must back up.");
            sendRaiseArm();
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

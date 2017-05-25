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
        recoverLockout = true;
        digPitchAngle = -4.0;
        for(int i=0; i<numDigsPerMine; i++)
        {
            sendDig(digPitchAngle);
            sendShake();
            digPitchAngle -= 1.0;
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
                          || ((robotStatus.yPos + robotCenterToScoopLength*sin(DEG2RAD*robotStatus.heading)) >= (DIG_MAP_Y_LEN - mapYOffset - miningWallBufferDistance))
                              || ((robotStatus.yPos + robotCenterToScoopLength*sin(DEG2RAD*robotStatus.heading)) <= miningWallBufferDistance - mapYOffset)
                               && (execInfoMsg.actionDeque.at(0) == 1 || execInfoMsg.actionDeque.at(0) == 2 || execInfoMsg.actionDeque.at(0) == 3 || execInfoMsg.actionDeque.at(0) == 13));
        if(tooCloseToWallLatch.LE_Latch(tooCloseToWall) && !sentTooCloseToWall)
        {
            ROS_WARN("Scoop too close to wall. Must back up.");
            sendDequeClearFront();
            sendRaiseArm(true);
            sendDriveRel(backUpDistance, 0.0, false, 0.0, true, true);
            sentTooCloseToWall = true;
            backUpFromWallSerialNum = serialNum;
        }
        if(sentTooCloseToWall && (execLastProcType == procType && execLastSerialNum == backUpFromWallSerialNum))
        {
            sentTooCloseToWall = false;
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
        recoverLockout = false;
        procsBeingExecuted[procType] = false;
        procsToExecute[procType] = false;
        procsToResume[procType] = false;
        state = _init_;
        break;
    }
}

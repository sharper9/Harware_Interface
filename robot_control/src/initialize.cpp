#include <robot_control/initialize.h>

bool Initialize::runProc()
{
    switch(state)
    {
    case _init_:
        procsBeingExecuted[procType] = true;
        procsToExecute[procType] = false;
        procsToResume[procType] = false;
        computeDriveSpeeds();
        state = _exec_;
        stage = _moveActuator;
        nextStage = _startTimer;
        sendPartiallyRaiseArm();
        initComplete = false;
        driveDeltaDistance = 0.2; // m
        rotateDeltaAngle = 15.0; // deg
        bucketRaised = false;
        resetQueueEmptyCondition();
        break;
    case _exec_:
        procsBeingExecuted[procType] = true;
        procsToExecute[procType] = false;
        procsToResume[procType] = false;
        computeDriveSpeeds();
        switch(stage)
        {
        case _startTimer:
            startupTime = ros::Time::now().toSec();
            performFullPoseUpdate = true;
            stage = _checkFullPose;
            break;
        case _checkFullPose:
            if(robotStatus.initialFullPoseFound)
            {
                if(moderateQualityInitPoseFound)
                {
                    xPosToUse = moderateQualityInitX;
                    yPosToUse = moderateQualityInitY;
                    headingToUse = fmod(fmod(moderateQualityInitHeading,360.0)+360.0,360.0);
                    if(headingToUse>180.0) headingToUse -= 360.0;
                }
                else
                {
                    xPosToUse = robotStatus.xPos;
                    yPosToUse = robotStatus.yPos;
                    headingToUse = fmod(fmod(robotStatus.heading,360.0)+360.0,360.0);
                    if(headingToUse>180.0) headingToUse -= 360.0;
                }
                performAManeuver = knownPositionCheckManeuver();
                if(raiseScoopFullyBeforeManeuver) sendRaiseArm();
                if(performAManeuver)
                {
                    if(driveDeltaDistance>0.0) sendDriveRel(driveDeltaDistance, rotateDeltaAngle, false, 0.0, false, false);
                    else sendDriveRel(fabs(driveDeltaDistance), rotateDeltaAngle, false, 0.0, false, true);
                    if(!raiseScoopFullyBeforeManeuver) sendRaiseArm();
                }
                stage = _moveActuator;
                if(robotStatus.fullPoseFound) nextStage = _completeInit;
                else nextStage = _startTimer;
            }
            else if((ros::Time::now().toSec() - startupTime) > waitForFullPoseTime)
            {
                if(bucketRaised)
                {
                    sendLowerBucket();
                    bucketRaised = false;
                    driveDeltaDistance = unknownPoseManeuvers.at(badInitPoseManeuverToPerform).driveDistance;
                    rotateDeltaAngle = unknownPoseManeuvers.at(badInitPoseManeuverToPerform).turnAngle;
                    if(driveDeltaDistance>0.0) sendDriveRel(driveDeltaDistance, rotateDeltaAngle, false, 0.0, false, false);
                    else sendDriveRel(fabs(driveDeltaDistance), rotateDeltaAngle, false, 0.0, false, true);
                }
                else
                {
                    sendRaiseBucket();
                    bucketRaised = true;
                }
                stage = _moveActuator;
                nextStage = _startTimer;
            }
            else stage = _checkFullPose;
            break;
        case _moveActuator:
            if((execLastProcType == procType && execLastSerialNum == serialNum) || queueEmptyTimedOut) stage = nextStage;
            else stage = _moveActuator;
            break;
        case _completeInit:
            initComplete = true;
            break;
        }
        if(initComplete) state = _finish_;
        else state = _exec_;
        serviceQueueEmptyCondition();
        break;
    case _interrupt_:
        procsBeingExecuted[procType] = false;
        procsToInterrupt[procType] = false;
        state = _exec_;
        break;
    case _finish_:
        initialized = true;
        procsBeingExecuted[procType] = false;
        procsToExecute[procType] = false;
        procsToResume[procType] = false;
        state = _finish_;
        break;
    }
}

bool Initialize::knownPositionCheckManeuver()
{
    driveDeltaDistance = 0.0;
    rotateDeltaAngle = 0.0;
    raiseScoopFullyBeforeManeuver = false;

    if(yPosToUse < -1.0 * baseStationDistance / 2.0) //in the leftmost zone
    {
        if(headingToUse <= -90.0 && headingToUse >= -180.0) //TODO: Test the -90 position and close measures
        {
            driveDeltaDistance = -0.3;
            return true;
        }
        else if(headingToUse > -90.0 && headingToUse <= 0.0)
        {
            ROS_WARN("INIT, CHECK_MANEUVER, LEFTMOST ZONE: I do not know how to fix this one exactly, here's a guess");
            //TODO: I do not know how to fix this one exactly, here's a guess
            rotateDeltaAngle = -5.0;
            driveDeltaDistance = -0.3;
            return true;
        }
        else if (headingToUse >= 90.0 && headingToUse <= 180.0)
        {
            rotateDeltaAngle = 5.0;
            driveDeltaDistance = -0.3;
            return true;
        }
        else
        {
            raiseScoopFullyBeforeManeuver = true;
            driveDeltaDistance = 0.3;
            return true;
        }
    }
    else if(yPosToUse > baseStationDistance / 2.0) // in the rightmost zone
    {
        if(headingToUse <= -90.0 && headingToUse >= -180.0)
        {
            rotateDeltaAngle = -5.0;
            driveDeltaDistance = -0.3;
            return true;
        }
        else if(headingToUse > -90.0 && headingToUse <= 0.0)
        {
            raiseScoopFullyBeforeManeuver = true;
            driveDeltaDistance = 0.3;
            return true;
        }
        else if (headingToUse >= 90.0 && headingToUse <= 180.0)
        {
            driveDeltaDistance = -0.3;
            return true;
        }
        else
        {
            ROS_WARN("INIT, CHECK_MANEUVER, RIGHTMOST ZONE: I do not know how to fix this one exactly, here's a guess");
            //TODO: I do not know how to fix this one exactly, here's a guess
            rotateDeltaAngle = 5.0;
            driveDeltaDistance = -0.3;
            return true;
        }
    }
    else //in the middle
    {
        if(headingToUse < -91.0 || headingToUse > 91.0)
        {
            driveDeltaDistance = -0.3;
            return true;
        }
        else if(xPosToUse < 0.5)
        {
            if(headingToUse <= 95.0 && headingToUse >= 85.0)
            {
                rotateDeltaAngle = 3.0;
                driveDeltaDistance = -0.3;
                return true;
            }
            else
            {
                rotateDeltaAngle = -3.0;
                driveDeltaDistance = -0.3;
                return true;
            }
        }
        else
        {
            raiseScoopFullyBeforeManeuver = true;
            return false;
        }
    }
}

#include <robot_control/initialize.h>

Initialize::Initialize()
{
    armRaised = false;
    actionList.resize(NUM_INIT_ACTIONS);
    // 0
    actionList.at(0).emplace_back(InitAction(__straight, 0.3));
    actionList.at(0).emplace_back(InitAction(__turn, -10.0));

    // 1
    actionList.at(1).emplace_back(InitAction(__armRaise));
    actionList.at(1).emplace_back(InitAction(__straight, 0.3));

    // 2
    actionList.at(2).emplace_back(InitAction(__turn, 10.0));

    // 3
    actionList.at(3).emplace_back(InitAction(__turn, 10.0));
    actionList.at(3).emplace_back(InitAction(__straight, -0.3));
    actionList.at(3).emplace_back(InitAction(__armRaise));
    actionList.at(3).emplace_back(InitAction(__straight, 0.3));
    actionList.at(3).emplace_back(InitAction(__turn, -20.0));

    // 4
    actionList.at(4).emplace_back(InitAction(__straight, -0.3));

    // 5
    actionList.at(5).emplace_back(InitAction(__turn, -10.0));

    // 6
    actionList.at(6).emplace_back(InitAction(__turn, -10.0));
    actionList.at(6).emplace_back(InitAction(__straight, -0.3));
    actionList.at(6).emplace_back(InitAction(__armRaise));
    actionList.at(6).emplace_back(InitAction(__straight, 0.3));
    actionList.at(6).emplace_back(InitAction(__turn, 20.0));

    // 7
    actionList.at(7).emplace_back(InitAction(__armRaise));
    actionList.at(7).emplace_back(InitAction(__turn, 10.0));

    // 8
    actionList.at(8).emplace_back(InitAction(__armRaise));
    actionList.at(8).emplace_back(InitAction(__turn, 10.0));
    actionList.at(8).emplace_back(InitAction(__straight, -0.3));
    actionList.at(8).emplace_back(InitAction(__turn, -20.0));
    actionList.at(8).emplace_back(InitAction(__straight, 0.3));

    // 9
    actionList.at(9).emplace_back(InitAction(__armRaise));
    actionList.at(9).emplace_back(InitAction(__turn, -10.0));

    // 10
    actionList.at(10).emplace_back(InitAction(__armRaise));
    actionList.at(10).emplace_back(InitAction(__turn, -10.0));
    actionList.at(10).emplace_back(InitAction(__straight, -0.3));
    actionList.at(10).emplace_back(InitAction(__turn, 20.0));
    actionList.at(10).emplace_back(InitAction(__straight, 0.3));

    // 11
    actionList.at(11).emplace_back(InitAction(__straight, 0.3));
    actionList.at(11).emplace_back(InitAction(__turn, 10.0));

    // 12
    actionList.at(12).emplace_back(InitAction(__straight, -0.3));
    actionList.at(12).emplace_back(InitAction(__turn, 45.0));

    // 13
    actionList.at(13).emplace_back(InitAction(__armRaise));
    actionList.at(13).emplace_back(InitAction(__turn, -10.0));
    actionList.at(13).emplace_back(InitAction(__straight, -0.3));

    // 14
    actionList.at(14).emplace_back(InitAction(__armRaise));
    actionList.at(14).emplace_back(InitAction(__turn, 10.0));
    actionList.at(14).emplace_back(InitAction(__straight, -0.3));
}

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
        sendPartiallyRaiseArm(false);
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
            ROS_INFO("_startTimer");
            startupTime = ros::Time::now().toSec();
            performFullPoseUpdate = true;
            stage = _checkFullPose;
            break;
        case _checkFullPose:
            if(robotStatus.initialFullPoseFound)
            {
                ROS_INFO("initial full pose found");
                // TODO: finish this
                if(raiseScoopFullyBeforeManeuver) sendRaiseArm(false);
                if(performAManeuver)
                {
                    if(driveDeltaDistance>0.0) sendDriveRel(driveDeltaDistance, rotateDeltaAngle, false, 0.0, false, false);
                    else sendDriveRel(fabs(driveDeltaDistance), rotateDeltaAngle, false, 0.0, false, true);
                    if(!raiseScoopFullyBeforeManeuver) sendRaiseArm(false);
                }
                stage = _moveActuator;
                if(robotStatus.fullPoseFound) nextStage = _completeInit;
                else nextStage = _startTimer;
            }
            else if((ros::Time::now().toSec() - startupTime) > waitForFullPoseTime)
            {
                ROS_INFO("check full pose timeout");
                if(bucketRaised)
                {
                    ROS_INFO("lower bucket and perform maneuver");
                    sendLowerBucket();
                    bucketRaised = false;
                    driveDeltaDistance = unknownPoseManeuvers.at(badInitPoseManeuverToPerform).driveDistance;
                    rotateDeltaAngle = unknownPoseManeuvers.at(badInitPoseManeuverToPerform).turnAngle;
                    if(driveDeltaDistance>0.0) sendDriveRel(driveDeltaDistance, rotateDeltaAngle, false, 0.0, false, false);
                    else sendDriveRel(fabs(driveDeltaDistance), rotateDeltaAngle, false, 0.0, false, true);
                }
                else
                {
                    ROS_INFO("raise bucket");
                    sendRaiseBucket();
                    bucketRaised = true;
                }
                stage = _moveActuator;
                nextStage = _startTimer;
            }
            else stage = _checkFullPose;
            break;
        case _moveActuator:
            ROS_INFO("move actuator");
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

Initialize::InitAction::InitAction(INIT_ACTION_TYPE_T actionTypeIn)
{
    actionType = actionTypeIn;
    parameter = 0.0;
}

Initialize::InitAction::InitAction(INIT_ACTION_TYPE_T actionTypeIn, float parameterIn)
{
    actionType = actionTypeIn;
    parameter = parameterIn;
}

#include <robot_control/initialize.h>

Initialize::Initialize()
{
    // 1, 7, 9, 12, (15)
    armRaised = false;
    actionList.resize(NUM_INIT_ACTIONS);
    // 0
    actionList.at(0).push_back(InitAction(__straight, 0.3));
    actionList.at(0).push_back(InitAction(__turn, -20.0));

    // 1
    actionList.at(1).push_back(InitAction(__armRaise));
    actionList.at(1).push_back(InitAction(__straight, 0.3));

    // 2
    actionList.at(2).push_back(InitAction(__turn, 20.0));

    // 3
    actionList.at(3).push_back(InitAction(__turn, 20.0));
    actionList.at(3).push_back(InitAction(__straight, -0.3));
    actionList.at(3).push_back(InitAction(__armRaise));
    actionList.at(3).push_back(InitAction(__straight, 0.3));
    actionList.at(3).push_back(InitAction(__turn, -30.0));

    // 4
    actionList.at(4).push_back(InitAction(__straight, -0.3));

    // 5
    actionList.at(5).push_back(InitAction(__turn, -20.0));

    // 6
    actionList.at(6).push_back(InitAction(__turn, -20.0));
    actionList.at(6).push_back(InitAction(__straight, -0.3));
    actionList.at(6).push_back(InitAction(__armRaise));
    actionList.at(6).push_back(InitAction(__straight, 0.3));
    actionList.at(6).push_back(InitAction(__turn, 30.0));

    // 7
    actionList.at(7).push_back(InitAction(__armRaise));
    actionList.at(7).push_back(InitAction(__turn, 20.0));

    // 8
    actionList.at(8).push_back(InitAction(__armRaise));
    actionList.at(8).push_back(InitAction(__turn, 20.0));
    actionList.at(8).push_back(InitAction(__straight, -0.3));
    actionList.at(8).push_back(InitAction(__turn, -30.0));
    actionList.at(8).push_back(InitAction(__straight, 0.3));

    // 9
    actionList.at(9).push_back(InitAction(__armRaise));
    actionList.at(9).push_back(InitAction(__turn, -20.0));

    // 10
    actionList.at(10).push_back(InitAction(__armRaise));
    actionList.at(10).push_back(InitAction(__turn, -20.0));
    actionList.at(10).push_back(InitAction(__straight, -0.3));
    actionList.at(10).push_back(InitAction(__turn, 30.0));
    actionList.at(10).push_back(InitAction(__straight, 0.3));

    // 11
    actionList.at(11).push_back(InitAction(__straight, 0.3));
    actionList.at(11).push_back(InitAction(__turn, 20.0));

    // 12
    actionList.at(12).push_back(InitAction(__straight, -0.3));
    actionList.at(12).push_back(InitAction(__armRaise));
    actionList.at(12).push_back(InitAction(__turn, 45.0));

    // 13
    actionList.at(13).push_back(InitAction(__armRaise));
    actionList.at(13).push_back(InitAction(__turn, -20.0));
    actionList.at(13).push_back(InitAction(__straight, -0.3));

    // 14
    actionList.at(14).push_back(InitAction(__armRaise));
    actionList.at(14).push_back(InitAction(__turn, 20.0));
    actionList.at(14).push_back(InitAction(__straight, -0.3));
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
                pushInitActionsToPerform();
                if(performAManeuver)
                {
                    stage = _moveActuator;
                    nextStage = _startTimer;
                }
                else stage = _completeInit;
            }
            else if((ros::Time::now().toSec() - startupTime) > waitForFullPoseTime)
            {
                ROS_INFO("check full pose timeout");
                pushInitActionsToPerform();
                if(performAManeuver)
                {
                    stage = _moveActuator;
                    nextStage = _startTimer;
                }
                else stage = _completeInit;
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

void Initialize::pushInitActionsToPerform()
{
    if(initManeuverToPerform==NUM_INIT_ACTIONS)
    {
        performAManeuver = false;
    }
    else
    {
        performAManeuver = true;
        for(int i=0; i<actionList.at(initManeuverToPerform).size(); i++)
        {
            actionList.at(initManeuverToPerform).at(i).pushAction();
        }
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

void Initialize::InitAction::pushAction()
{
    switch(actionType)
    {
    case __straight:
        if(parameter>=0.0) sendDriveRel(parameter, 0.0, false, 0.0, false, false);
        else sendDriveRel(parameter, 0.0, false, 0.0, false, true);
        break;
    case __turn:
        ROS_INFO("turn parameter = %f",parameter);
        sendDriveRel(0.0, parameter, false, 0.0, false, false);
        break;
    case __armRaise:
        sendRaiseArm(false);
        break;
    case __bucketRaisePartially:
        sendPartiallyRaiseBucket();
        break;
    case __bucketLower:
        sendLowerBucket();
        break;
    default:
        ROS_ERROR("Initialize, InitAction, invalid action type pushed = %i",static_cast<int>(actionType));
        break;
    }
}

bool Initialize::InitAction::runProc()
{
    return true;
}

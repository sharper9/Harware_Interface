#include <robot_control/initialize.h>

Initialize::Initialize()
{
    // 1, 7, 9, 12, (15)
    armRaised = false;
    actionList.resize(NUM_INIT_ACTIONS);
    // 0
    actionList.at(0).push_back(InitAction(__straight, 0.4, &this->serialNum));
    actionList.at(0).push_back(InitAction(__turn, -20.0, &this->serialNum));

    // 1
    actionList.at(1).push_back(InitAction(__armRaise, &this->serialNum));
    actionList.at(1).push_back(InitAction(__straight, 0.4, &this->serialNum));

    // 2
    actionList.at(2).push_back(InitAction(__turn, 20.0, &this->serialNum));

    // 3
    actionList.at(3).push_back(InitAction(__turn, 20.0, &this->serialNum));
    actionList.at(3).push_back(InitAction(__straight, -0.4, &this->serialNum));
    actionList.at(3).push_back(InitAction(__armRaise, &this->serialNum));
    actionList.at(3).push_back(InitAction(__straight, 0.4, &this->serialNum));
    actionList.at(3).push_back(InitAction(__turn, -30.0, &this->serialNum));

    // 4
    actionList.at(4).push_back(InitAction(__straight, -0.4, &this->serialNum));

    // 5
    actionList.at(5).push_back(InitAction(__turn, -20.0, &this->serialNum));

    // 6
    actionList.at(6).push_back(InitAction(__turn, -20.0, &this->serialNum));
    actionList.at(6).push_back(InitAction(__straight, -0.4, &this->serialNum));
    actionList.at(6).push_back(InitAction(__armRaise, &this->serialNum));
    actionList.at(6).push_back(InitAction(__straight, 0.4, &this->serialNum));
    actionList.at(6).push_back(InitAction(__turn, 30.0, &this->serialNum));

    // 7
    actionList.at(7).push_back(InitAction(__armRaise, &this->serialNum));
    actionList.at(7).push_back(InitAction(__turn, 20.0, &this->serialNum));

    // 8
    actionList.at(8).push_back(InitAction(__armRaise, &this->serialNum));
    actionList.at(8).push_back(InitAction(__turn, 20.0, &this->serialNum));
    actionList.at(8).push_back(InitAction(__straight, -0.4, &this->serialNum));
    actionList.at(8).push_back(InitAction(__turn, -30.0, &this->serialNum));
    actionList.at(8).push_back(InitAction(__straight, 0.4, &this->serialNum));

    // 9
    actionList.at(9).push_back(InitAction(__armRaise, &this->serialNum));
    actionList.at(9).push_back(InitAction(__turn, -20.0, &this->serialNum));

    // 10
    actionList.at(10).push_back(InitAction(__armRaise, &this->serialNum));
    actionList.at(10).push_back(InitAction(__turn, -20.0, &this->serialNum));
    actionList.at(10).push_back(InitAction(__straight, -0.4, &this->serialNum));
    actionList.at(10).push_back(InitAction(__turn, 30.0, &this->serialNum));
    actionList.at(10).push_back(InitAction(__straight, 0.4, &this->serialNum));

    // 11
    actionList.at(11).push_back(InitAction(__straight, 0.4, &this->serialNum));
    actionList.at(11).push_back(InitAction(__turn, 20.0, &this->serialNum));

    // 12
    actionList.at(12).push_back(InitAction(__straight, -0.4, &this->serialNum));
    actionList.at(12).push_back(InitAction(__armRaise, &this->serialNum));
    actionList.at(12).push_back(InitAction(__turn, 45.0, &this->serialNum));

    // 13
    actionList.at(13).push_back(InitAction(__armRaise, &this->serialNum));
    actionList.at(13).push_back(InitAction(__turn, -20.0, &this->serialNum));
    actionList.at(13).push_back(InitAction(__straight, -0.4, &this->serialNum));

    // 14
    actionList.at(14).push_back(InitAction(__armRaise, &this->serialNum));
    actionList.at(14).push_back(InitAction(__turn, 20.0, &this->serialNum));
    actionList.at(14).push_back(InitAction(__straight, -0.4, &this->serialNum));

    // 15 (no maneuver)

    // 16
    actionList.at(16).push_back(InitAction(__armRaise, &this->serialNum));
    actionList.at(16).push_back(InitAction(__turn, -130.0, &this->serialNum));

    // 17
    actionList.at(17).push_back(InitAction(__armRaise, &this->serialNum));
    actionList.at(17).push_back(InitAction(__turn, 90.0, &this->serialNum));

    // 18
    actionList.at(18).push_back(InitAction(__armRaise, &this->serialNum));
    actionList.at(18).push_back(InitAction(__turn, 40.0, &this->serialNum));

    // 19
    actionList.at(19).push_back(InitAction(__armRaise, &this->serialNum));
    actionList.at(19).push_back(InitAction(__turn, -40.0, &this->serialNum));

    // 20
    actionList.at(20).push_back(InitAction(__armRaise, &this->serialNum));
    actionList.at(20).push_back(InitAction(__turn, -90.0, &this->serialNum));

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
        //stage = _moveActuator;
        stage = _startTimer;
        //nextStage = _startTimer;
        //actionList.at(0).at(0).setProcType(this->procType);
        actionList.at(0).at(0).sendPartiallyRaiseArm(false);
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
            ROS_INFO("start timer");
            startupTime = ros::Time::now().toSec();
            performFullPoseUpdate = true;
            resetQueueEmptyCondition();
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
                    nextStage = _completeInit;
                }
                else stage = _completeInit;
            }
            else if((ros::Time::now().toSec() - startupTime) > waitForFullPoseTime)
            {
                ROS_INFO("wait for full pose timeout");
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
            if((execLastProcType == procType && execLastSerialNum == serialNum) || queueEmptyTimedOut) stage = nextStage;
            else stage = _moveActuator;
            break;
        case _completeInit:
            ROS_INFO("complete init");
            initComplete = true;
            break;
        }
        if(initComplete)
        {
            sendRaiseArm(false);
            state = _finish_;
        }
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
    if(initManeuverToPerform==INIT_COMPLETE_NUM)
    {
        performAManeuver = false;
    }
    else
    {
        performAManeuver = true;
        for(int i=0; i<actionList.at(initManeuverToPerform).size(); i++)
        {
            actionList.at(initManeuverToPerform).at(i).pushAction(this->procType);
        }
    }
}

Initialize::InitAction::InitAction(INIT_ACTION_TYPE_T actionTypeIn, unsigned int* initializeSerialNumPtrIn)
{
    actionType = actionTypeIn;
    parameter = 0.0;
    initializeSerialNumPtr = initializeSerialNumPtrIn;
}

Initialize::InitAction::InitAction(INIT_ACTION_TYPE_T actionTypeIn, float parameterIn, unsigned int* initializeSerialNumPtrIn)
{
    actionType = actionTypeIn;
    parameter = parameterIn;
    initializeSerialNumPtr = initializeSerialNumPtrIn;
}

void Initialize::InitAction::pushAction(PROC_TYPES_T procTypeIn)
{
    this->procType = procTypeIn;
    this->serialNum = *initializeSerialNumPtr;
    switch(actionType)
    {
    case __straight:
        if(parameter>=0.0) sendDriveRel(parameter, 0.0, false, 0.0, false, false);
        else sendDriveRel(parameter, 0.0, false, 0.0, false, true);
        break;
    case __turn:
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
    *initializeSerialNumPtr = this->serialNum;
}

void Initialize::InitAction::setProcType(PROC_TYPES_T procTypeIn)
{
    this->procType = procTypeIn;
}

bool Initialize::InitAction::runProc()
{
    return true;
}

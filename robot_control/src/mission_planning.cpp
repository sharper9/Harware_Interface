#include <robot_control/mission_planning.h>

MissionPlanning::MissionPlanning()
{
    execActionClient = nh.serviceClient<messages::ExecAction>("control/exec/actionin");
    ExecActionEndedSub = nh.subscribe<messages::ExecActionEnded>("control/exec/actionended", 1, &MissionPlanning::ExecActionEndedCallback_, this);
    execInfoSub = nh.subscribe<messages::ExecInfo>("control/exec/info", 1, &MissionPlanning::execInfoCallback_, this);
    navSub = nh.subscribe<messages::NavFilterOut>("navigation/navigationfilterout/navigationfilterout", 1, &MissionPlanning::navCallback_, this);
    controlServ = nh.advertiseService("/control/missionplanning/control", &MissionPlanning::controlCallback_, this);
    infoPub = nh.advertise<messages::MissionPlanningInfo>("/control/missionplanning/info", 1);
    driveSpeedsPub = nh.advertise<robot_control::DriveSpeeds>("/control/missionplanning/drivespeeds", 1);
    multiProcLockout = false;
    lockoutSum = 0;
    initialized = false;
    atMineLocation = false;
    bucketFull = false;
    atDepositLocation = false;
    confirmedAtDepositLocation = false;
    stuck = false;
    pauseStarted = false;
    robotStatus.pauseSwitch = true;
    execDequeEmpty = true;
    execLastSerialNum = 99;
    initialize.reg(__initialize__);
    driveToDig.reg(__driveToDig__);
    mine.reg(__mine__);
    driveToDeposit.reg(__driveToDeposit__);
    deposit.reg(__deposit__);
    recover.reg(__recover__);
    missionTime = 0.0;
    prevTime = ros::Time::now().toSec();
    timers[_queueEmptyTimer_] = new CataglyphisTimer<MissionPlanning>(&MissionPlanning::queueEmptyTimerCallback_, this);
    timers[_queueEmptyTimer_]->setPeriod(queueEmptyTimerPeriod);
    timers[_queueEmptyTimer_]->stop();
    driveSpeedsMsg.vMax = defaultVMax;
    driveSpeedsMsg.rMax = defaultRMax;
    driveSpeedsMsgPrev.vMax = 0.0;
    driveSpeedsMsgPrev.rMax = 0.0;
    driveSpeedsPub.publish(driveSpeedsMsg);
    infoMsg.procsToExecute.resize(NUM_PROC_TYPES,0);
    infoMsg.procsToInterrupt.resize(NUM_PROC_TYPES,0);
    infoMsg.procsBeingExecuted.resize(NUM_PROC_TYPES,0);
    infoMsg.procsToResume.resize(NUM_PROC_TYPES,0);
    infoMsg.procStates.resize(NUM_PROC_TYPES,_init_);
    srand(time(NULL));
    for(int i=0; i<NUM_PROC_TYPES; i++)
    {
        procsToExecute[i] = false;
        procsToInterrupt[i] = false;
        procsBeingExecuted[i] = false;
        procsToResume[i] = false;
    }
}

void MissionPlanning::run()
{
    robotStatus.pauseSwitch = false; // **********!!!!!!!!!! Need to figure out where this comes from
    ROS_INFO_THROTTLE(3,"Mission Planning running...");
    evalConditions_();
    ROS_DEBUG("robotStatus.pauseSwitch = %i",robotStatus.pauseSwitch);
    if(robotStatus.pauseSwitch) runPause_();
    else runProcedures_();
    packAndPubInfoMsg_();
    if(missionStarted && !robotStatus.pauseSwitch) missionTime += ros::Time::now().toSec() - prevTime;
    prevTime = ros::Time::now().toSec();
}

void MissionPlanning::evalConditions_()
{
    if(multiProcLockout)
    {
        multiProcLockout = true;
        robotStatus.pauseSwitch = true;
        ROS_FATAL_THROTTLE(3,"tried to execute multiple procedures..........");
    }
    else
    {
        calcnumProcsBeingOrToBeExecOrRes_();
        if(numProcsBeingOrToBeExecOrRes==0 && !initialized && !robotStatus.pauseSwitch) // Initialize
        {
            procsToExecute[__initialize__] = true;
            ROS_INFO("to execute initialize");
        }
        calcnumProcsBeingOrToBeExecOrRes_();
        if(numProcsBeingOrToBeExecOrRes==0 && initialized && !atMineLocation && !bucketFull && !atDepositLocation && !stuck) // DriveToDig
        {
            procsToExecute[__driveToDig__] = true;
            ROS_INFO("to execute driveToDig");
        }
        calcnumProcsBeingOrToBeExecOrRes_();
        if(numProcsBeingOrToBeExecOrRes==0 && initialized && atMineLocation && !bucketFull && !atDepositLocation && !stuck) // Mine
        {
            procsToExecute[__mine__] = true;
            ROS_INFO("to execute mine");
        }
        calcnumProcsBeingOrToBeExecOrRes_();
        if(numProcsBeingOrToBeExecOrRes==0 && initialized && bucketFull && !atDepositLocation && !stuck) // DriveToDeposit
        {
            procsToExecute[__driveToDeposit__] = true;
            ROS_INFO("to execute driveToDeposit");
        }
        calcnumProcsBeingOrToBeExecOrRes_();
        if(numProcsBeingOrToBeExecOrRes==0 && initialized && bucketFull && atDepositLocation && confirmedAtDepositLocation && !stuck) // Deposit
        {
            procsToExecute[__deposit__] = true;
            ROS_INFO("to execute deposit");
        }
        calcnumProcsBeingOrToBeExecOrRes_();
        if(numProcsBeingOrToBeExecOrRes==0 && initialized && stuck)
        {
            procsToExecute[__recover__] = true;
            ROS_INFO("to execute recover");
        }
        calcnumProcsBeingOrToBeExecOrRes_();
        if((numProcsBeingOrToBeExecOrRes==0 || numProcsToBeExecAndNotInterrupt>1) && initialized) // "Fallthrough" condition
        {
            initialized = true;
            atMineLocation = false;
            bucketFull = false; // *** not sure about this
            atDepositLocation = false;
            stuck = false;
        }

        // *************** Multi Proc Lockout for testing *************************
        /*lockoutSum = 0;
        for(int i=0; i<NUM_PROC_TYPES; i++) if(procsToExecute[i] && !procsToInterrupt[i]) lockoutSum++;
        if(lockoutSum>1) multiProcLockout = true;
        else multiProcLockout = false;
        if(multiProcLockout)
        {
            robotStatus.pauseSwitch = true;
            ROS_FATAL("tried to execute multiple procedures..........");
            voiceSay->call("tried to execute multiple procedures. tisk tisk.");
        }*/
        // *************************************************************************
    }
}

void MissionPlanning::runProcedures_()
{
    if(pauseStarted == true)
    {
        pause.sendUnPause();
        resumeTimers_();
        if(missionStarted==false) missionStarted = true;
    }
    pauseStarted = false;
    initialize.run();
    driveToDig.run();
    mine.run();
    driveToDeposit.run();
    recover.run();
}

void MissionPlanning::runPause_()
{
    if(pauseStarted == false)
    {
        pause.sendPause();
        pauseAllTimers_();
    }
    pauseStarted = true;
}

void MissionPlanning::pauseAllTimers_()
{
    for(int i=0; i<NUM_TIMERS; i++) if(timers[i]->running) timers[i]->pause();
}

void MissionPlanning::resumeTimers_()
{
    for(int i=0; i<NUM_TIMERS; i++) if(timers[i]->running) timers[i]->resume();
}

void MissionPlanning::calcnumProcsBeingOrToBeExecOrRes_()
{
    numProcsBeingOrToBeExecOrRes = 0;
    numProcsBeingOrToBeExec = 0;
    numProcsToBeExecAndNotInterrupt = 0;
    for(int i=0; i<NUM_PROC_TYPES; i++) 
    {
        if(procsBeingExecuted[i] || procsToExecute[i] || procsToResume[i]) numProcsBeingOrToBeExecOrRes++;
        if(procsBeingExecuted[i] || procsToExecute[i]) numProcsBeingOrToBeExec++;
        if(procsToExecute[i] && !procsToInterrupt[i]) numProcsToBeExecAndNotInterrupt++;
    }
}

void MissionPlanning::packAndPubInfoMsg_()
{
    infoMsg.initialized = initialized;
    infoMsg.atMineLocation = atMineLocation;
    infoMsg.bucketFull = bucketFull;
    infoMsg.atDepositLocation = atDepositLocation;
    infoMsg.stuck = stuck;
    infoMsg.pause = robotStatus.pauseSwitch;
    infoMsg.numProcs = NUM_PROC_TYPES;
    for(int i=0; i<NUM_PROC_TYPES; i++)
    {
        infoMsg.procsToExecute.at(i) = procsToExecute[i];
        infoMsg.procsToInterrupt.at(i) = procsToInterrupt[i];
        infoMsg.procsBeingExecuted.at(i) = procsBeingExecuted[i];
        infoMsg.procsToResume.at(i) = procsToResume[i];
        switch(static_cast<PROC_TYPES_T>(i))
        {
        case __initialize__:
            infoMsg.procStates.at(i) = initialize.state;
            break;
        case __driveToDig__:
            infoMsg.procStates.at(i) = driveToDig.state;
            break;
        case __mine__:
            infoMsg.procStates.at(i) = mine.state;
            break;
        case __driveToDeposit__:
            infoMsg.procStates.at(i) = driveToDeposit.state;
            break;
        case __recover__:
            infoMsg.procStates.at(i) = recover.state;
            break;
        }
    }
    infoMsg.missionTime = missionTime;
    infoPub.publish(infoMsg);
}

void MissionPlanning::ExecActionEndedCallback_(const messages::ExecActionEnded::ConstPtr &msg)
{
    execDequeEmpty = msg->dequeEmpty;
    execLastProcType = static_cast<PROC_TYPES_T>(msg->procType);
    execLastSerialNum = msg->serialNum;
}

void MissionPlanning::navCallback_(const messages::NavFilterOut::ConstPtr &msg)
{
    robotStatus.rollAngle = msg->roll;
    robotStatus.pitchAngle = msg->pitch;
    robotStatus.xPos = msg->x_position;
    robotStatus.yPos = msg->y_position;
    robotStatus.heading = msg->heading;
}

void MissionPlanning::execInfoCallback_(const messages::ExecInfo::ConstPtr &msg)
{
    execInfoMsg = *msg;
}

bool MissionPlanning::controlCallback_(messages::MissionPlanningControl::Request &req, messages::MissionPlanningControl::Response &res)
{
    initialized = req.initialized;
    atMineLocation = req.atMineLocation;
    bucketFull = req.bucketFull;
    atDepositLocation = req.atDepositLocation;
    stuck = req.stuck;
    for(int i=0; i<req.numProcs; i++)
    {
        procsToExecute[i] = req.procsToExecute.at(i);
        procsToInterrupt[i] = req.procsToInterrupt.at(i);
        procsBeingExecuted[i] = req.procsBeingExecuted.at(i);
        procsToResume[i] = req.procsToResume.at(i);
        switch(static_cast<PROC_TYPES_T>(i)) // If PROC_TYPES_T enum is ever edited, edit this as well
        {
        case __initialize__:
            initialize.state = static_cast<PROC_STATE_T>(req.procStates.at(i));
            break;
        case __driveToDig__:
            driveToDig.state = static_cast<PROC_STATE_T>(req.procStates.at(i));
            break;
        case __mine__:
            mine.state = static_cast<PROC_STATE_T>(req.procStates.at(i));
            break;
        case __driveToDeposit__:
            driveToDeposit.state = static_cast<PROC_STATE_T>(req.procStates.at(i));
            break;
        case __deposit__:
            deposit.state = static_cast<PROC_STATE_T>(req.procStates.at(i));
            break;
        }
    }
    missionTime = req.missionTime;
    return true;
}

void MissionPlanning::queueEmptyTimerCallback_(const ros::TimerEvent &event)
{
    queueEmptyTimedOut = true;
    ROS_WARN("queue empty timer expired");
}

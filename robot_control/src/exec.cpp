#include <robot_control/exec.h>

Exec::Exec()
{
	robotStatus.loopRate = loopRate;
    actionServ = nh.advertiseService("control/exec/actionin", &Exec::actionCallback_, this);
    manualOverrideServ = nh.advertiseService("/control/exec/manualoverride", &Exec::manualOverrideCallback_, this);
    navSub = nh.subscribe<messages::NavFilterOut>("navigation/navigationfilterout/navigationfilterout", 1, &Exec::navCallback_, this);
    scoopSub = nh.subscribe<hw_interface_plugin_roboteq::Roboteq_Data>("/roboteq/brushed/scoop", 1, &Exec::scoopCallback_, this);
    armSub = nh.subscribe<hw_interface_plugin_roboteq::Roboteq_Data>("/roboteq/brushed/arm", 1, &Exec::armCallback_, this);
    bucketSub = nh.subscribe<hw_interface_plugin_roboteq::Roboteq_Data>("/roboteq/brushed/bucket", 1, &Exec::bucketCallback_, this);
    driveSpeedsSub = nh.subscribe<robot_control::DriveSpeeds>("/control/missionplanning/drivespeeds", 1, &Exec::driveSpeedsCallback_, this);
	actuatorPub = nh.advertise<messages::ActuatorOut>("control/actuatorout/all",1);
	infoPub = nh.advertise<messages::ExecInfo>("control/exec/info",1);
    actionEndedPub = nh.advertise<messages::ExecActionEnded>("control/exec/actionended",1);
	// "allocate" deque memory
    for(int i=0; i<NUM_ACTIONS; i++)
    {
        actionPoolIndex_[i] = 0;
    }
	for(int j=0; j<ACTION_POOL_SIZE; j++)
    {
        actionPool_[_idle][j] = new Idle;
		actionPool_[_driveGlobal][j] = new DriveGlobal;
		actionPool_[_driveRelative][j] = new DriveRelative;
        actionPool_[_dig][j] = new Dig;
        actionPool_[_dump][j] = new Dump;
        actionPool_[_wait][j] = new Wait;
	}
	for(int k=0; k<NUM_TASKS; k++)
	{
		pauseIdle_.taskPoolIndex[k] = 0;
	}
	for(int l=0; l<TASK_POOL_SIZE; l++)
	{
		pauseIdle_.taskPool[_driveHalt_][l] = new DriveHalt;
		pauseIdle_.taskPool[_driveStraight_][l] = new DriveStraight;
		pauseIdle_.taskPool[_pivot_][l] = new DrivePivot;
        //pauseIdle_.taskPool[_driveArc_][l] = new DriveArc;
        pauseIdle_.taskPool[_scoopHalt_][l] = new ScoopHalt;
        pauseIdle_.taskPool[_scoopSetPos_][l] = new ScoopSetPos;
        pauseIdle_.taskPool[_armHalt_][l] = new ArmHalt;
        pauseIdle_.taskPool[_armSetPos_][l] = new ArmSetPos;
        pauseIdle_.taskPool[_armShake_][l] = new ArmShake;
        pauseIdle_.taskPool[_bucketHalt_][l] = new BucketHalt;
        pauseIdle_.taskPool[_bucketSetPos_][l] = new BucketSetPos;
	}
    execStartTime_ = ros::Time::now().toSec();
    actionDequeEmptyPrev_ = true;
}

void Exec::run()
{
    ROS_INFO_THROTTLE(3,"Exec running...");
    if(clearDequeFlag_) {actionDeque_.clear(); pauseIdle_.clearDeques();} // Clear deques
    if(clearFrontFlag_) currentActionDone_ = 1; // Artificially set currentActionDone_ to true so that it is popped off the deque to clear it
    if(newActionFlag_) // New action to be added to deque
    {
        if(pushToFrontFlag_) // Push new action to front of deque
        {
            actionDeque_.push_front(actionPool_[nextActionType_][actionPoolIndex_[nextActionType_]]); // Push new action pointer of specified type to front of deque
            actionDeque_.front()->params = params_; // Assign params into action object just pushed
        }
        else // Push new action to back of deque
        {
            actionDeque_.push_back(actionPool_[nextActionType_][actionPoolIndex_[nextActionType_]]); // Push new action pointer of specified type to back of deque
            actionDeque_.back()->params = params_; // Assign params into action object just pushed
        }
        actionPoolIndex_[nextActionType_]++; // Increment the action pool index of the action type just pushed
        if(actionPoolIndex_[nextActionType_]>=ACTION_POOL_SIZE) actionPoolIndex_[nextActionType_] = 0; // If pool index has wrapped around, restart at 0
    }
    if(pause_==true && pausePrev_==false) pauseIdle_.driveHalt.init(); // Call init on driveHalt to begin possible drive hold
	if(pause_)
	{
        ROS_INFO_THROTTLE(3,"exec pause");
		pauseIdle_.run(); // If pause switch is true, run pause action
		if(pushToFrontFlag_ || (newActionFlag_ && actionDeque_.size()==1)) actionDeque_.front()->init();
	}
    else // Else, run actions from deque
    {
        if(actionDeque_.empty()) // Check if deque is empty
        {
			pauseIdle_.run(); // Perform pause action to halt the robot
            currentActionDone_ = 0;
        }
        else // Else, deque is not empty and the action at the front() should be run
        {
			if(pushToFrontFlag_ || (newActionFlag_ && actionDeque_.size()==1)) actionDeque_.front()->init(); // If new action was pushed to the front or deque was empty and became not empty, need to run init() for new action
            if(currentActionDone_) // If the action at the front of the deque is complete, pop this action off and init() the next action in the deque
            {
                execActionEndedMsgOut_.procType = static_cast<uint8_t>(actionDeque_.front()->params.procType); // Record proc type of action that just ended
                execActionEndedMsgOut_.serialNum = actionDeque_.front()->params.serialNum; // Record serial number of action that just ended
                actionDeque_.pop_front(); // Pop completed action off of front of deque
                if(!actionDeque_.empty()) {actionDeque_.front()->init(); execActionEndedMsgOut_.dequeEmpty = false;} // If the deque is not empty, then the next action, now on the front, needs to be initialized
                else execActionEndedMsgOut_.dequeEmpty = true; // Else, the deque is empty and the deque empty flag is set to true
                actionEndedPub.publish(execActionEndedMsgOut_); // Publish the action ended message with the proc type and serial number so Mission Planning knows the action ended
            }
            if(!actionDeque_.empty()) currentActionDone_ = actionDeque_.front()->run(); // If the deque is not empty, call the run method on the action on the front of the deque
            else currentActionDone_ = 0; // Else, the deque is empty and no action is running
        }
    }
    actionDequeEmptyPrev_ = actionDeque_.empty();
    clearDequeFlag_ = false;
    clearFrontFlag_ = false;
    newActionFlag_ = false;
    pushToFrontFlag_ = false;
	pausePrev_ = pause_;
	packActuatorMsgOut_();
	packInfoMsgOut_();
    if(!manualOverride_)
    {
        actuatorPub.publish(actuatorMsgOut_);
        infoPub.publish(execInfoMsgOut_);
    }
    //std::printf("\n");
    /*execElapsedTime_ = ros::Time::now().toSec() - execStartTime_;
    ROS_INFO("*******\nexecElapsedTime = %f",execElapsedTime_);
    for(int i=0; i<NUM_ACTIONS; i++) ROS_INFO("actionPoolIndex[%i] = %i",i,actionPoolIndex_[i]);
    for(int i=0; i<NUM_TASKS; i++) ROS_INFO("taskPoolIndex[%i] = %i",i,pauseIdle_.taskPoolIndex[i]);
    ROS_INFO("actionDeque size = %u",actionDeque_.size());
    ROS_INFO("driveDeque size = %u",pauseIdle_.driveDeque.size());
    ROS_INFO("grabberDeque size = %u",pauseIdle_.grabberDeque.size());
    ROS_INFO("visionDeque size = %u\n",pauseIdle_.visionDeque.size());*/
}

bool Exec::actionCallback_(messages::ExecAction::Request &req, messages::ExecAction::Response &res)
{
    nextActionType_ = static_cast<ACTION_TYPE_T>(req.nextActionType);
    newActionFlag_ = req.newActionFlag;
    pushToFrontFlag_ = req.pushToFrontFlag;
    clearDequeFlag_ = req.clearDequeFlag;
    clearFrontFlag_ = req.clearFrontFlag;
    if(!req.pauseUnchanged) pause_ = req.pause;
	params_.actionType = nextActionType_;
    params_.float1 = req.float1;
    params_.float2 = req.float2;
    params_.float3 = req.float3;
    params_.float4 = req.float4;
    params_.float5 = req.float5;
    params_.int1 = req.int1;
    params_.bool1 = req.bool1;
    params_.bool2 = req.bool2;
    params_.bool3 = req.bool3;
    params_.bool4 = req.bool4;
    params_.bool5 = req.bool5;
    params_.bool6 = req.bool6;
    params_.bool7 = req.bool7;
    params_.bool8 = req.bool8;
    params_.procType = static_cast<PROC_TYPES_T>(req.procType);
    params_.serialNum = req.serialNum;
    //ROS_INFO("ACTION CALLBACK,\n\t nextActionType = %i,\n\t newActionFlag = %i,\n\t pushToFrontFlag = %i,\n\t clearDequeFlag = %i,\n\t clearFrontFlag = %i,\n\t pause = %i,\n\t float1 = %f,\n\t float2 = %f",
    //         nextActionType_, newActionFlag_, pushToFrontFlag_, clearDequeFlag_, clearFrontFlag_, pause_, params_.float1,params_.float2);
    return true;
}

bool Exec::manualOverrideCallback_(messages::ExecManualOverride::Request &req, messages::ExecManualOverride::Response &res)
{
    manualOverride_ = req.manualOverride;
    return true;
}

void Exec::navCallback_(const messages::NavFilterOut::ConstPtr &msg)
{
    robotStatus.yawRate = msg->yaw_rate;
    robotStatus.rollAngle = msg->roll;
    robotStatus.pitchAngle = msg->pitch;
    robotStatus.velocity = msg->velocity;
    robotStatus.deltaDistance = msg->delta_distance;
    robotStatus.heading = msg->heading;
    robotStatus.xPos = msg->x_position;
    robotStatus.yPos = msg->y_position;
}

void Exec::scoopCallback_(const hw_interface_plugin_roboteq::Roboteq_Data::ConstPtr& msg)
{
    // TODO: correctly set feedback
    //robotStatus.scoopStatus = msg->destination_reached[0] && msg->destination_reached[1];
    //robotStatus.scoopPos = (msg->analog_inputs[0] + msg->analog_inputs[1])/2.0; // !!! This may not be the right way to get the feedback position...
}

void Exec::armCallback_(const hw_interface_plugin_roboteq::Roboteq_Data::ConstPtr& msg)
{
    // TODO: correctly set feedback
    //robotStatus.armStatus = msg->destination_reached[0] && msg->destination_reached[1];
    //robotStatus.armPos = (msg->analog_inputs[0] + msg->analog_inputs[1])/2.0; // !!! This may not be the right way to get the feedback position...
}

void Exec::bucketCallback_(const hw_interface_plugin_roboteq::Roboteq_Data::ConstPtr& msg)
{
    // TODO: correctly set feedback
    //robotStatus.bucketStatus = msg->destination_reached[0] && msg->destination_reached[1];
    //robotStatus.bucketPos = (msg->analog_inputs[0] + msg->analog_inputs[1])/2.0; // !!! This may not be the right way to get the feedback position...
}

void Exec::driveSpeedsCallback_(const robot_control::DriveSpeeds::ConstPtr &msg)
{
    robotStatus.vMax = msg->vMax;
    robotStatus.rMax = msg->rMax;
}

void Exec::packActuatorMsgOut_()
{
	actuatorMsgOut_.fl_speed_cmd = robotOutputs.flMotorSpeed;
	actuatorMsgOut_.fr_speed_cmd = robotOutputs.frMotorSpeed;
	actuatorMsgOut_.bl_speed_cmd = robotOutputs.blMotorSpeed;
	actuatorMsgOut_.br_speed_cmd = robotOutputs.brMotorSpeed;
    actuatorMsgOut_.wrist_pos_cmd = robotOutputs.scoopPosCmd;
    actuatorMsgOut_.arm_pos_cmd = robotOutputs.armPosCmd;
    actuatorMsgOut_.bucket_pos_cmd = robotOutputs.bucketPosCmd;
    actuatorMsgOut_.wrist_stop_cmd = robotOutputs.scoopStopCmd;
    actuatorMsgOut_.arm_stop_cmd = robotOutputs.armStopCmd;
    actuatorMsgOut_.bucket_stop_cmd = robotOutputs.bucketStopCmd;
}

void Exec::packInfoMsgOut_()
{
	execInfoMsgOut_.actionDequeSize = actionDeque_.size();
	execInfoMsgOut_.pause = pause_;
    execInfoMsgOut_.stopFlag = robotOutputs.stopFlag;
    execInfoMsgOut_.turnFlag = robotOutputs.turnFlag;
	for(unsigned int i=0; i<100; i++)
	{
		execInfoMsgOut_.actionDeque[i] = 0;
		execInfoMsgOut_.actionFloat1[i] = 0;
		execInfoMsgOut_.actionFloat2[i] = 0;
		execInfoMsgOut_.actionFloat3[i] = 0;
		execInfoMsgOut_.actionFloat4[i] = 0;
		execInfoMsgOut_.actionFloat5[i] = 0;
		execInfoMsgOut_.actionInt1[i] = 0;
		execInfoMsgOut_.actionBool1[i] = 0;
        execInfoMsgOut_.actionBool2[i] = 0;
        execInfoMsgOut_.actionBool3[i] = 0;
        execInfoMsgOut_.actionBool4[i] = 0;
        execInfoMsgOut_.actionBool5[i] = 0;
        execInfoMsgOut_.actionBool6[i] = 0;
        execInfoMsgOut_.actionBool7[i] = 0;
        execInfoMsgOut_.actionBool8[i] = 0;
        execInfoMsgOut_.actionProcType[i] = 0;
        execInfoMsgOut_.actionSerialNum[i] = 0;
	}
	for(unsigned int i=0; i<execInfoMsgOut_.actionDequeSize; i++)
	{
		execInfoMsgOut_.actionDeque[i] = actionDeque_.at(i)->params.actionType;
		execInfoMsgOut_.actionFloat1[i] = actionDeque_.at(i)->params.float1;
		execInfoMsgOut_.actionFloat2[i] = actionDeque_.at(i)->params.float2;
		execInfoMsgOut_.actionFloat3[i] = actionDeque_.at(i)->params.float3;
		execInfoMsgOut_.actionFloat4[i] = actionDeque_.at(i)->params.float4;
		execInfoMsgOut_.actionFloat5[i] = actionDeque_.at(i)->params.float5;
		execInfoMsgOut_.actionInt1[i] = actionDeque_.at(i)->params.int1;
		execInfoMsgOut_.actionBool1[i] = actionDeque_.at(i)->params.bool1;
        execInfoMsgOut_.actionBool2[i] = actionDeque_.at(i)->params.bool2;
        execInfoMsgOut_.actionBool3[i] = actionDeque_.at(i)->params.bool3;
        execInfoMsgOut_.actionBool4[i] = actionDeque_.at(i)->params.bool4;
        execInfoMsgOut_.actionBool5[i] = actionDeque_.at(i)->params.bool5;
        execInfoMsgOut_.actionBool6[i] = actionDeque_.at(i)->params.bool6;
        execInfoMsgOut_.actionBool7[i] = actionDeque_.at(i)->params.bool7;
        execInfoMsgOut_.actionBool8[i] = actionDeque_.at(i)->params.bool8;
        execInfoMsgOut_.actionProcType[i] = static_cast<uint8_t>(actionDeque_.at(i)->params.procType);
        execInfoMsgOut_.actionSerialNum[i] = actionDeque_.at(i)->params.serialNum;
	}
}

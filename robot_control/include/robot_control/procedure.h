#ifndef PROCEDURE_H
#define PROCEDURE_H
#include "mission_planning_procedure_share.h"

class Procedure : public MissionPlanningProcedureShare
{
public:
    // Members
    PROC_TYPES_T procType;
    PROC_STATE_T state;
	unsigned int serialNum = 0;
	bool dequeClearFront = false;
    // Methods
    void reg(PROC_TYPES_T procTypeIn);
    bool run();
    virtual bool runProc() = 0;
    void clearAndResizeWTT();
	void sendDriveGlobal(bool pushToFront, bool endHeadingFlag, float endHeading, bool driveBackwards);
    void sendDriveRel(float deltaDistance, float deltaHeading, bool endHeadingFlag, float endHeading, bool frontOfDeque, bool driveBackwards);
    void sendDriveToWall();
    void sendDig(float goalPitchAngle);
    void sendDump();
	void sendWait(float waitTime, bool pushToFront); // sec
    void sendRaiseArm(bool pushToFront);
    void sendPartiallyRaiseArm(bool pushToFront);
    void sendRaiseBucket();
    void sendLowerBucket();
    void sendPrepareArmBucket();
    void sendPartiallyRaiseBucket();
    void sendShake();
	void sendDequeClearFront();
	void sendDequeClearAll();
	void sendPause();
	void sendUnPause();
	void computeDriveSpeeds();
	void resetQueueEmptyCondition();
	void serviceQueueEmptyCondition();
};

void Procedure::reg(PROC_TYPES_T procTypeIn)
{
    this->procType = procTypeIn;
    this->state = _init_;
}

bool Procedure::run()
{
    //ROS_DEBUG("before if(procsToExecute.at(this->procType");
    //ROS_DEBUG("this->procType = %i",static_cast<int>(this->procType));
	if(procsToInterrupt[this->procType] && this->state==_exec_) {this->state = _interrupt_; procsToResume[this->procType] = true;}
	if((procsToResume[this->procType] && numProcsBeingOrToBeExec==0) || (procsToExecute[this->procType] || procsBeingExecuted[this->procType])) return this->runProc();
	//else if(procsBeingExecuted.at(this->procType) == true && procsToExecute.at(this->procType) == false) {this->state = _interrupt_; return this->runProc();}
    //ROS_DEBUG("after if - else if(procsToExecute.at(this->procType");
}

void Procedure::clearAndResizeWTT()
{
    waypointsToTravel.clear();
    waypointsToTravel.resize(numWaypointsToTravel);
}

void Procedure::sendDriveGlobal(bool pushToFront, bool endHeadingFlag, float endHeading, bool driveBackwards)
{
    for(int i=0; i<numWaypointsToTravel; i++)
    {
		this->serialNum++;
        execActionSrv.request.nextActionType = _driveGlobal;
        execActionSrv.request.newActionFlag = 1;
		execActionSrv.request.pushToFrontFlag = pushToFront;
        execActionSrv.request.clearDequeFlag = false;
		execActionSrv.request.clearFrontFlag = false;
        execActionSrv.request.pause = false;
		execActionSrv.request.pauseUnchanged = true;
		execActionSrv.request.float1 = waypointsToTravel.at(i).x;
		execActionSrv.request.float2 = waypointsToTravel.at(i).y;
		execActionSrv.request.float3 = endHeading;
		execActionSrv.request.float4 = 0.0;
        execActionSrv.request.float5 = 0.0;
		execActionSrv.request.int1 = 0;
		if(i==(numWaypointsToTravel-1)) execActionSrv.request.bool1 = endHeadingFlag;
		else execActionSrv.request.bool1 = false;
		execActionSrv.request.bool2 = pushToFront;
		execActionSrv.request.bool3 = driveBackwards;
		execActionSrv.request.bool4 = false;
		execActionSrv.request.bool5 = false;
		execActionSrv.request.bool6 = false;
		execActionSrv.request.bool7 = false;
		execActionSrv.request.bool8 = false;
        execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
        execActionSrv.request.serialNum = this->serialNum;
        if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
        else ROS_ERROR("exec action service call unsuccessful");
    }
}

void Procedure::sendDriveRel(float deltaDistance, float deltaHeading, bool endHeadingFlag, float endHeading, bool frontOfDeque, bool driveBackwards)
{
	this->serialNum++;
    execActionSrv.request.nextActionType = _driveRelative;
    execActionSrv.request.newActionFlag = 1;
    execActionSrv.request.pushToFrontFlag = frontOfDeque;
    execActionSrv.request.clearDequeFlag = false;
	execActionSrv.request.clearFrontFlag = false;
    execActionSrv.request.pause = false;
	execActionSrv.request.pauseUnchanged = true;
    execActionSrv.request.float1 = deltaDistance;
    execActionSrv.request.float2 = deltaHeading;
	execActionSrv.request.float3 = endHeading;
	execActionSrv.request.float4 = 0.0;
	execActionSrv.request.float5 = 0.0;
    execActionSrv.request.int1 = 0;
	execActionSrv.request.bool1 = endHeadingFlag;
	execActionSrv.request.bool2 = frontOfDeque;
    execActionSrv.request.bool3 = driveBackwards;
	execActionSrv.request.bool4 = false;
	execActionSrv.request.bool5 = false;
	execActionSrv.request.bool6 = false;
	execActionSrv.request.bool7 = false;
	execActionSrv.request.bool8 = false;
    execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
    execActionSrv.request.serialNum = this->serialNum;
    if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
    else ROS_ERROR("exec action service call unsuccessful");
}

void Procedure::sendDriveToWall()
{
    this->serialNum++;
    execActionSrv.request.nextActionType = _driveToWall;
    execActionSrv.request.newActionFlag = 1;
    execActionSrv.request.pushToFrontFlag = false;
    execActionSrv.request.clearDequeFlag = false;
    execActionSrv.request.clearFrontFlag = false;
    execActionSrv.request.pause = false;
    execActionSrv.request.pauseUnchanged = true;
    execActionSrv.request.float1 = 0.0;
    execActionSrv.request.float2 = 0.0;
    execActionSrv.request.float3 = 0.0;
    execActionSrv.request.float4 = 0.0;
    execActionSrv.request.float5 = 0.0;
    execActionSrv.request.int1 = 0;
    execActionSrv.request.bool1 = false;
    execActionSrv.request.bool2 = false;
    execActionSrv.request.bool3 = false;
    execActionSrv.request.bool4 = false;
    execActionSrv.request.bool5 = false;
    execActionSrv.request.bool6 = false;
    execActionSrv.request.bool7 = false;
    execActionSrv.request.bool8 = false;
    execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
    execActionSrv.request.serialNum = this->serialNum;
    if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
    else ROS_ERROR("exec action service call unsuccessful");
}

void Procedure::sendDig(float goalPitchAngle)
{
	this->serialNum++;
    execActionSrv.request.nextActionType = _dig;
	execActionSrv.request.newActionFlag = 1;
	execActionSrv.request.pushToFrontFlag = false;
	execActionSrv.request.clearDequeFlag = false;
	execActionSrv.request.clearFrontFlag = false;
	execActionSrv.request.pause = false;
	execActionSrv.request.pauseUnchanged = true;
    execActionSrv.request.float1 = goalPitchAngle;
	execActionSrv.request.float2 = 0.0;
	execActionSrv.request.float3 = 0.0;
	execActionSrv.request.float4 = 0.0;
	execActionSrv.request.float5 = 0.0;
	execActionSrv.request.int1 = 0;
	execActionSrv.request.bool1 = false;
	execActionSrv.request.bool2 = false;
	execActionSrv.request.bool3 = false;
	execActionSrv.request.bool4 = false;
	execActionSrv.request.bool5 = false;
	execActionSrv.request.bool6 = false;
	execActionSrv.request.bool7 = false;
	execActionSrv.request.bool8 = false;
	execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
	execActionSrv.request.serialNum = this->serialNum;
	if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
	else ROS_ERROR("exec action service call unsuccessful");
}

void Procedure::sendDump()
{
	this->serialNum++;
    execActionSrv.request.nextActionType = _dump;
	execActionSrv.request.newActionFlag = 1;
	execActionSrv.request.pushToFrontFlag = false;
	execActionSrv.request.clearDequeFlag = false;
	execActionSrv.request.clearFrontFlag = false;
	execActionSrv.request.pause = false;
	execActionSrv.request.pauseUnchanged = true;
	execActionSrv.request.float1 = 0.0;
	execActionSrv.request.float2 = 0.0;
	execActionSrv.request.float3 = 0.0;
	execActionSrv.request.float4 = 0.0;
	execActionSrv.request.float5 = 0.0;
	execActionSrv.request.int1 = 0;
	execActionSrv.request.bool1 = false;
	execActionSrv.request.bool2 = false;
	execActionSrv.request.bool3 = false;
	execActionSrv.request.bool4 = false;
	execActionSrv.request.bool5 = false;
	execActionSrv.request.bool6 = false;
	execActionSrv.request.bool7 = false;
	execActionSrv.request.bool8 = false;
	execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
	execActionSrv.request.serialNum = this->serialNum;
	if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
	else ROS_ERROR("exec action service call unsuccessful");
}

void Procedure::sendWait(float waitTime, bool pushToFront)
{
	this->serialNum++;
	execActionSrv.request.nextActionType = _wait;
	execActionSrv.request.newActionFlag = 1;
	execActionSrv.request.pushToFrontFlag = pushToFront;
	execActionSrv.request.clearDequeFlag = false;
	execActionSrv.request.clearFrontFlag = false;
	execActionSrv.request.pause = false;
	execActionSrv.request.pauseUnchanged = true;
	execActionSrv.request.float1 = waitTime;
	execActionSrv.request.float2 = 0.0;
	execActionSrv.request.float3 = 0.0;
	execActionSrv.request.float4 = 0.0;
	execActionSrv.request.float5 = 0.0;
	execActionSrv.request.int1 = 0;
	execActionSrv.request.bool1 = false;
	execActionSrv.request.bool2 = false;
	execActionSrv.request.bool3 = false;
	execActionSrv.request.bool4 = false;
	execActionSrv.request.bool5 = false;
	execActionSrv.request.bool6 = false;
	execActionSrv.request.bool7 = false;
	execActionSrv.request.bool8 = false;
	execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
	execActionSrv.request.serialNum = this->serialNum;
	if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
	else ROS_ERROR("exec action service call unsuccessful");
}

void Procedure::sendRaiseArm(bool pushToFront)
{
    this->serialNum++;
    execActionSrv.request.nextActionType = _raiseArm;
    execActionSrv.request.newActionFlag = 1;
    execActionSrv.request.pushToFrontFlag = pushToFront;
    execActionSrv.request.clearDequeFlag = false;
    execActionSrv.request.clearFrontFlag = false;
    execActionSrv.request.pause = false;
    execActionSrv.request.pauseUnchanged = true;
    execActionSrv.request.float1 = 0.0;
    execActionSrv.request.float2 = 0.0;
    execActionSrv.request.float3 = 0.0;
    execActionSrv.request.float4 = 0.0;
    execActionSrv.request.float5 = 0.0;
    execActionSrv.request.int1 = 0;
    execActionSrv.request.bool1 = false;
    execActionSrv.request.bool2 = false;
    execActionSrv.request.bool3 = false;
    execActionSrv.request.bool4 = false;
    execActionSrv.request.bool5 = false;
    execActionSrv.request.bool6 = false;
    execActionSrv.request.bool7 = false;
    execActionSrv.request.bool8 = false;
    execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
    execActionSrv.request.serialNum = this->serialNum;
    if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
    else ROS_ERROR("exec action service call unsuccessful");
}

void Procedure::sendPartiallyRaiseArm(bool pushToFront)
{
    this->serialNum++;
    execActionSrv.request.nextActionType = _partiallyRaiseArm;
    execActionSrv.request.newActionFlag = 1;
    execActionSrv.request.pushToFrontFlag = pushToFront;
    execActionSrv.request.clearDequeFlag = false;
    execActionSrv.request.clearFrontFlag = false;
    execActionSrv.request.pause = false;
    execActionSrv.request.pauseUnchanged = true;
    execActionSrv.request.float1 = 0.0;
    execActionSrv.request.float2 = 0.0;
    execActionSrv.request.float3 = 0.0;
    execActionSrv.request.float4 = 0.0;
    execActionSrv.request.float5 = 0.0;
    execActionSrv.request.int1 = 0;
    execActionSrv.request.bool1 = false;
    execActionSrv.request.bool2 = false;
    execActionSrv.request.bool3 = false;
    execActionSrv.request.bool4 = false;
    execActionSrv.request.bool5 = false;
    execActionSrv.request.bool6 = false;
    execActionSrv.request.bool7 = false;
    execActionSrv.request.bool8 = false;
    execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
    execActionSrv.request.serialNum = this->serialNum;
    if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
    else ROS_ERROR("exec action service call unsuccessful");
}

void Procedure::sendRaiseBucket()
{
    this->serialNum++;
    execActionSrv.request.nextActionType = _raiseBucket;
    execActionSrv.request.newActionFlag = 1;
    execActionSrv.request.pushToFrontFlag = false;
    execActionSrv.request.clearDequeFlag = false;
    execActionSrv.request.clearFrontFlag = false;
    execActionSrv.request.pause = false;
    execActionSrv.request.pauseUnchanged = true;
    execActionSrv.request.float1 = 0.0;
    execActionSrv.request.float2 = 0.0;
    execActionSrv.request.float3 = 0.0;
    execActionSrv.request.float4 = 0.0;
    execActionSrv.request.float5 = 0.0;
    execActionSrv.request.int1 = 0;
    execActionSrv.request.bool1 = false;
    execActionSrv.request.bool2 = false;
    execActionSrv.request.bool3 = false;
    execActionSrv.request.bool4 = false;
    execActionSrv.request.bool5 = false;
    execActionSrv.request.bool6 = false;
    execActionSrv.request.bool7 = false;
    execActionSrv.request.bool8 = false;
    execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
    execActionSrv.request.serialNum = this->serialNum;
    if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
    else ROS_ERROR("exec action service call unsuccessful");
}

void Procedure::sendLowerBucket()
{
    this->serialNum++;
    execActionSrv.request.nextActionType = _lowerBucket;
    execActionSrv.request.newActionFlag = 1;
    execActionSrv.request.pushToFrontFlag = false;
    execActionSrv.request.clearDequeFlag = false;
    execActionSrv.request.clearFrontFlag = false;
    execActionSrv.request.pause = false;
    execActionSrv.request.pauseUnchanged = true;
    execActionSrv.request.float1 = 0.0;
    execActionSrv.request.float2 = 0.0;
    execActionSrv.request.float3 = 0.0;
    execActionSrv.request.float4 = 0.0;
    execActionSrv.request.float5 = 0.0;
    execActionSrv.request.int1 = 0;
    execActionSrv.request.bool1 = false;
    execActionSrv.request.bool2 = false;
    execActionSrv.request.bool3 = false;
    execActionSrv.request.bool4 = false;
    execActionSrv.request.bool5 = false;
    execActionSrv.request.bool6 = false;
    execActionSrv.request.bool7 = false;
    execActionSrv.request.bool8 = false;
    execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
    execActionSrv.request.serialNum = this->serialNum;
    if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
    else ROS_ERROR("exec action service call unsuccessful");
}

void Procedure::sendPrepareArmBucket()
{
    this->serialNum++;
    execActionSrv.request.nextActionType = _prepareArmBucket;
    execActionSrv.request.newActionFlag = 1;
    execActionSrv.request.pushToFrontFlag = false;
    execActionSrv.request.clearDequeFlag = false;
    execActionSrv.request.clearFrontFlag = false;
    execActionSrv.request.pause = false;
    execActionSrv.request.pauseUnchanged = true;
    execActionSrv.request.float1 = 0.0;
    execActionSrv.request.float2 = 0.0;
    execActionSrv.request.float3 = 0.0;
    execActionSrv.request.float4 = 0.0;
    execActionSrv.request.float5 = 0.0;
    execActionSrv.request.int1 = 0;
    execActionSrv.request.bool1 = false;
    execActionSrv.request.bool2 = false;
    execActionSrv.request.bool3 = false;
    execActionSrv.request.bool4 = false;
    execActionSrv.request.bool5 = false;
    execActionSrv.request.bool6 = false;
    execActionSrv.request.bool7 = false;
    execActionSrv.request.bool8 = false;
    execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
    execActionSrv.request.serialNum = this->serialNum;
    if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
    else ROS_ERROR("exec action service call unsuccessful");
}

void Procedure::sendPartiallyRaiseBucket()
{
    this->serialNum++;
    execActionSrv.request.nextActionType = _partiallyRaiseBucket;
    execActionSrv.request.newActionFlag = 1;
    execActionSrv.request.pushToFrontFlag = false;
    execActionSrv.request.clearDequeFlag = false;
    execActionSrv.request.clearFrontFlag = false;
    execActionSrv.request.pause = false;
    execActionSrv.request.pauseUnchanged = true;
    execActionSrv.request.float1 = 0.0;
    execActionSrv.request.float2 = 0.0;
    execActionSrv.request.float3 = 0.0;
    execActionSrv.request.float4 = 0.0;
    execActionSrv.request.float5 = 0.0;
    execActionSrv.request.int1 = 0;
    execActionSrv.request.bool1 = false;
    execActionSrv.request.bool2 = false;
    execActionSrv.request.bool3 = false;
    execActionSrv.request.bool4 = false;
    execActionSrv.request.bool5 = false;
    execActionSrv.request.bool6 = false;
    execActionSrv.request.bool7 = false;
    execActionSrv.request.bool8 = false;
    execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
    execActionSrv.request.serialNum = this->serialNum;
    if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
    else ROS_ERROR("exec action service call unsuccessful");
}

void Procedure::sendShake()
{
    this->serialNum++;
    execActionSrv.request.nextActionType = _shake;
    execActionSrv.request.newActionFlag = 1;
    execActionSrv.request.pushToFrontFlag = false;
    execActionSrv.request.clearDequeFlag = false;
    execActionSrv.request.clearFrontFlag = false;
    execActionSrv.request.pause = false;
    execActionSrv.request.pauseUnchanged = true;
    execActionSrv.request.float1 = 0.0;
    execActionSrv.request.float2 = 0.0;
    execActionSrv.request.float3 = 0.0;
    execActionSrv.request.float4 = 0.0;
    execActionSrv.request.float5 = 0.0;
    execActionSrv.request.int1 = 0;
    execActionSrv.request.bool1 = false;
    execActionSrv.request.bool2 = false;
    execActionSrv.request.bool3 = false;
    execActionSrv.request.bool4 = false;
    execActionSrv.request.bool5 = false;
    execActionSrv.request.bool6 = false;
    execActionSrv.request.bool7 = false;
    execActionSrv.request.bool8 = false;
    execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
    execActionSrv.request.serialNum = this->serialNum;
    if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
    else ROS_ERROR("exec action service call unsuccessful");
}

void Procedure::sendDequeClearFront()
{
	this->serialNum++;
	execActionSrv.request.nextActionType = 0;
	execActionSrv.request.newActionFlag = 0;
	execActionSrv.request.pushToFrontFlag = false;
	execActionSrv.request.clearDequeFlag = false;
	execActionSrv.request.clearFrontFlag = true;
	execActionSrv.request.pause = false;
	execActionSrv.request.pauseUnchanged = true;
	execActionSrv.request.float1 = 0.0;
	execActionSrv.request.float2 = 0.0;
	execActionSrv.request.float3 = 0.0;
	execActionSrv.request.float4 = 0.0;
	execActionSrv.request.float5 = 0.0;
	execActionSrv.request.int1 = 0;
	execActionSrv.request.bool1 = false;
	execActionSrv.request.bool2 = false;
	execActionSrv.request.bool3 = false;
	execActionSrv.request.bool4 = false;
	execActionSrv.request.bool5 = false;
	execActionSrv.request.bool6 = false;
	execActionSrv.request.bool7 = false;
	execActionSrv.request.bool8 = false;
	execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
	execActionSrv.request.serialNum = this->serialNum;
	if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
	else ROS_ERROR("exec action service call unsuccessful");
	ROS_INFO("send dequeClearFront");
	//voiceSay->call("queue clear front");
}

void Procedure::sendDequeClearAll()
{
	this->serialNum++;
	execActionSrv.request.nextActionType = 0;
	execActionSrv.request.newActionFlag = 0;
	execActionSrv.request.pushToFrontFlag = false;
	execActionSrv.request.clearDequeFlag = true;
	execActionSrv.request.clearFrontFlag = false;
	execActionSrv.request.pause = false;
	execActionSrv.request.pauseUnchanged = true;
	execActionSrv.request.float1 = 0.0;
	execActionSrv.request.float2 = 0.0;
	execActionSrv.request.float3 = 0.0;
	execActionSrv.request.float4 = 0.0;
	execActionSrv.request.float5 = 0.0;
	execActionSrv.request.int1 = 0;
	execActionSrv.request.bool1 = false;
	execActionSrv.request.bool2 = false;
	execActionSrv.request.bool3 = false;
	execActionSrv.request.bool4 = false;
	execActionSrv.request.bool5 = false;
	execActionSrv.request.bool6 = false;
	execActionSrv.request.bool7 = false;
	execActionSrv.request.bool8 = false;
	execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
	execActionSrv.request.serialNum = this->serialNum;
	if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
	else ROS_ERROR("exec action service call unsuccessful");
}

void Procedure::sendPause()
{
    this->serialNum++;
    execActionSrv.request.nextActionType = 0;
    execActionSrv.request.newActionFlag = 0;
    execActionSrv.request.pushToFrontFlag = false;
    execActionSrv.request.clearDequeFlag = false;
    execActionSrv.request.clearFrontFlag = false;
    execActionSrv.request.pause = true;
	execActionSrv.request.pauseUnchanged = false;
    execActionSrv.request.float1 = 0.0;
    execActionSrv.request.float2 = 0.0;
    execActionSrv.request.float3 = 0.0;
    execActionSrv.request.float4 = 0.0;
    execActionSrv.request.float5 = 0.0;
    execActionSrv.request.int1 = 0;
    execActionSrv.request.bool1 = false;
    execActionSrv.request.bool2 = false;
    execActionSrv.request.bool3 = false;
    execActionSrv.request.bool4 = false;
    execActionSrv.request.bool5 = false;
    execActionSrv.request.bool6 = false;
    execActionSrv.request.bool7 = false;
    execActionSrv.request.bool8 = false;
    execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
    execActionSrv.request.serialNum = this->serialNum;
    if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
    else ROS_ERROR("exec action service call unsuccessful");
}

void Procedure::sendUnPause()
{
    this->serialNum++;
    execActionSrv.request.nextActionType = 0;
    execActionSrv.request.newActionFlag = 0;
    execActionSrv.request.pushToFrontFlag = false;
    execActionSrv.request.clearDequeFlag = false;
    execActionSrv.request.clearFrontFlag = false;
    execActionSrv.request.pause = false;
	execActionSrv.request.pauseUnchanged = false;
    execActionSrv.request.float1 = 0.0;
    execActionSrv.request.float2 = 0.0;
    execActionSrv.request.float3 = 0.0;
    execActionSrv.request.float4 = 0.0;
    execActionSrv.request.float5 = 0.0;
    execActionSrv.request.int1 = 0;
    execActionSrv.request.bool1 = false;
    execActionSrv.request.bool2 = false;
    execActionSrv.request.bool3 = false;
    execActionSrv.request.bool4 = false;
    execActionSrv.request.bool5 = false;
    execActionSrv.request.bool6 = false;
    execActionSrv.request.bool7 = false;
    execActionSrv.request.bool8 = false;
    execActionSrv.request.procType = static_cast<uint8_t>(this->procType);
    execActionSrv.request.serialNum = this->serialNum;
    if(execActionClient.call(execActionSrv)) ROS_DEBUG("exec action service call successful");
    else ROS_ERROR("exec action service call unsuccessful");
}

void Procedure::computeDriveSpeeds()
{
    driveSpeedsMsg.vMax = defaultVMax;
    driveSpeedsMsg.rMax = defaultRMax;
	if((driveSpeedsMsg.vMax != driveSpeedsMsgPrev.vMax) || (driveSpeedsMsg.rMax != driveSpeedsMsgPrev.rMax)) driveSpeedsPub.publish(driveSpeedsMsg);
	driveSpeedsMsgPrev.vMax = driveSpeedsMsg.vMax;
	driveSpeedsMsgPrev.rMax = driveSpeedsMsg.rMax;
}

void Procedure::resetQueueEmptyCondition()
{
	timers[_queueEmptyTimer_]->stop();
	queueEmptyTimedOut = false;
	ROS_INFO("reset queue empty timer");
}

void Procedure::serviceQueueEmptyCondition()
{
	if(execInfoMsg.actionDequeSize==0 && !timers[_queueEmptyTimer_]->running && !queueEmptyTimedOut) {timers[_queueEmptyTimer_]->start(); ROS_WARN("start queue empty timer");}
	else if(execInfoMsg.actionDequeSize>0 && timers[_queueEmptyTimer_]->running) {timers[_queueEmptyTimer_]->stop(); queueEmptyTimedOut = false; ROS_INFO("stop queue empty timer");}
}

#endif // PROCEDURE_H

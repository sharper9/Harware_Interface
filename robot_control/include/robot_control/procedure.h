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
    std::vector<robot_control::Waypoint>::iterator intermWaypointsIt;
    int initNumWaypointsToTravel;
	int totalIntermWaypoints;
	bool dequeClearFront = false;
    // Methods
    void reg(PROC_TYPES_T procTypeIn);
    bool run();
    virtual bool runProc() = 0;
    void clearAndResizeWTT();
    void callIntermediateWaypoints();
	void sendDriveGlobal(bool pushToFront, bool endHeadingFlag, float endHeading);
	void sendDriveRel(float deltaDistance, float deltaHeading, bool endHeadingFlag, float endHeading, bool frontOfDeque);
	void sendGrab();
	void sendDrop();
	void sendOpen();
	void sendWait(float waitTime, bool pushToFront); // sec
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

void Procedure::callIntermediateWaypoints()
{
    initNumWaypointsToTravel = numWaypointsToTravel;
    totalIntermWaypoints = 0;
	intermediateWaypointsSrv.request.start_x = robotStatus.xPos;
	intermediateWaypointsSrv.request.start_y = robotStatus.yPos;
	intermediateWaypointsSrv.request.current_heading = robotStatus.heading;
	intermediateWaypointsSrv.request.waypointArrayIn.resize(numWaypointsToTravel);
	intermediateWaypointsSrv.request.waypointArrayIn = waypointsToTravel;
	if(intermediateWaypointsClient.call(intermediateWaypointsSrv)) ROS_DEBUG("intermediateWaypoints service call successful, size = %u", intermediateWaypointsSrv.response.waypointArrayOut.size());
	else ROS_ERROR("intermediateWaypoints service call unsuccessful");
	if(intermediateWaypointsSrv.response.waypointArrayOut.size() != numWaypointsToTravel)
	{
		waypointsToTravel.clear();
		waypointsToTravel = intermediateWaypointsSrv.response.waypointArrayOut;
		numWaypointsToTravel = waypointsToTravel.size();
		//for(int i=0; i<numWaypointsToTravel; i++) ROS_INFO("waypointsToTravel(%i) = (%f,%f)",i,waypointsToTravel.at(i).x,waypointsToTravel.at(i).y);
	}
	/*for(int i = 0; i < initNumWaypointsToTravel; i++)
    {
        intermWaypointsIt = waypointsToTravel.begin() + i + totalIntermWaypoints;
        if(i == 0)
        {
            intermediateWaypointsSrv.request.start.x = robotStatus.xPos;
            intermediateWaypointsSrv.request.start.y = robotStatus.yPos;
        }
        else
        {
            intermediateWaypointsSrv.request.start.x = (*(intermWaypointsIt - 1)).x;
            intermediateWaypointsSrv.request.start.y = (*(intermWaypointsIt - 1)).y;
        }
        intermediateWaypointsSrv.request.finish.x = (*intermWaypointsIt).x;
        intermediateWaypointsSrv.request.finish.y = (*intermWaypointsIt).y;
		if(intermediateWaypointsClient.call(intermediateWaypointsSrv)) ROS_DEBUG("intermediateWaypoints service call successful, size = %u", intermediateWaypointsSrv.response.waypointArray.size());
        else ROS_ERROR("intermediateWaypoints service call unsuccessful");
        if(intermediateWaypointsSrv.response.waypointArray.size() > 0)
        {
            waypointsToTravel.insert(intermWaypointsIt,intermediateWaypointsSrv.response.waypointArray.begin(),intermediateWaypointsSrv.response.waypointArray.end());
            totalIntermWaypoints += intermediateWaypointsSrv.response.waypointArray.size();
            numWaypointsToTravel += intermediateWaypointsSrv.response.waypointArray.size();
        }
	}*/
}

void Procedure::sendDriveGlobal(bool pushToFront, bool endHeadingFlag, float endHeading)
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
		execActionSrv.request.float6 = 0.0;
		execActionSrv.request.float7 = 0.0;
		execActionSrv.request.int1 = waypointsToTravel.at(i).maxAvoids;
		if(i==(numWaypointsToTravel-1)) execActionSrv.request.bool1 = endHeadingFlag;
		else execActionSrv.request.bool1 = false;
		execActionSrv.request.bool2 = pushToFront;
		execActionSrv.request.bool3 = waypointsToTravel.at(i).unskippable;
		execActionSrv.request.bool4 = waypointsToTravel.at(i).roiWaypoint;
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

void Procedure::sendDriveRel(float deltaDistance, float deltaHeading, bool endHeadingFlag, float endHeading, bool frontOfDeque)
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
	execActionSrv.request.float6 = 0.0;
	execActionSrv.request.float7 = 0.0;
    execActionSrv.request.int1 = 0;
	execActionSrv.request.bool1 = endHeadingFlag;
	execActionSrv.request.bool2 = frontOfDeque;
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

void Procedure::sendGrab()
{
	this->serialNum++;
	execActionSrv.request.nextActionType = _grab;
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
	execActionSrv.request.float6 = 0.0;
	execActionSrv.request.float7 = 0.0;
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

void Procedure::sendDrop()
{
	this->serialNum++;
	execActionSrv.request.nextActionType = _drop;
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
	execActionSrv.request.float6 = 0.0;
	execActionSrv.request.float7 = 0.0;
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

void Procedure::sendOpen()
{
	this->serialNum++;
	execActionSrv.request.nextActionType = _open;
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
	execActionSrv.request.float6 = 0.0;
	execActionSrv.request.float7 = 0.0;
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
	execActionSrv.request.float6 = 0.0;
	execActionSrv.request.float7 = 0.0;
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
	execActionSrv.request.float6 = 0.0;
	execActionSrv.request.float7 = 0.0;
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
	execActionSrv.request.float6 = 0.0;
	execActionSrv.request.float7 = 0.0;
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
	execActionSrv.request.float6 = 0.0;
	execActionSrv.request.float7 = 0.0;
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
	execActionSrv.request.float6 = 0.0;
	execActionSrv.request.float7 = 0.0;
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
	if(collisionMsg.slowdown)
	{
		driveSpeedsMsg.vMax = slowVMax;
		driveSpeedsMsg.rMax = defaultRMax;
	}
	else
	{
		driveSpeedsMsg.vMax = defaultVMax;
		driveSpeedsMsg.rMax = defaultRMax;
	}
	if((driveSpeedsMsg.vMax != driveSpeedsMsgPrev.vMax) || (driveSpeedsMsg.rMax != driveSpeedsMsgPrev.rMax)) driveSpeedsPub.publish(driveSpeedsMsg);
	driveSpeedsMsgPrev.vMax = driveSpeedsMsg.vMax;
	driveSpeedsMsgPrev.rMax = driveSpeedsMsg.rMax;
}

void Procedure::serviceAvoidCounterDecrement()
{
	if(hypot(robotStatus.xPos - prevAvoidCountDecXPos, robotStatus.yPos - prevAvoidCountDecYPos) > metersPerAvoidCountDecrement)
	{
		if(avoidCount > 0) avoidCount--;
		prevAvoidCountDecXPos = robotStatus.xPos;
		prevAvoidCountDecYPos = robotStatus.yPos;
	}
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

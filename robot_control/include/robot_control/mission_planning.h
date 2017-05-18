#ifndef MISSION_PLANNING_H
#define MISSION_PLANNING_H
#include <ros/ros.h>
#include "mission_planning_procedure_share.h"
#include <messages/ExecActionEnded.h>
#include <messages/MissionPlanningInfo.h>
#include <messages/MissionPlanningControl.h>
#include <messages/NavFilterOut.h>
#include <td_navigation/Td_navigation_Status.h>
#include <hw_interface_plugin_agent/pause.h>
#include "initialize.h"
#include "drive_to_dig.h"
#include "mine.h"
#include "drive_to_deposit.h"
#include "deposit_realign.h"
#include "deposit.h"
#include "recover.h"
#include "pause.h"
#include "bit_utils.h"

class MissionPlanning : public MissionPlanningProcedureShare
{
public:
	// Methods
	MissionPlanning();
	void run();
	// Members
	ros::NodeHandle nh;
	ros::Publisher infoPub;
	ros::Subscriber ExecActionEndedSub;
	ros::Subscriber navSub;
    ros::Subscriber pauseSub;
    ros::Subscriber tdNavStatusSub;
	ros::ServiceServer controlServ;
	messages::MissionPlanningInfo infoMsg;
	const int loopRate = 20; // Hz
    Initialize initialize;
    DriveToDig driveToDig;
    Mine mine;
    DriveToDeposit driveToDeposit;
    DepositRealign depositRealign;
    Deposit deposit;
    Recover recover;
    Pause pause;
	bool multiProcLockout;
	unsigned int lockoutSum;
	bool pauseStarted;
private:
	void evalConditions_();
	void runProcedures_();
	void runPause_();
	void pauseAllTimers_();
	void resumeTimers_();
	void calcnumProcsBeingOrToBeExecOrRes_();
	void packAndPubInfoMsg_();
	void initializeDigPlanningMap_();
	void ExecActionEndedCallback_(const messages::ExecActionEnded::ConstPtr& msg);
	void navCallback_(const messages::NavFilterOut::ConstPtr& msg);
	void execInfoCallback_(const messages::ExecInfo::ConstPtr& msg);
    void pauseCallback_(const hw_interface_plugin_agent::pause::ConstPtr& msg);
    void tdNavStatusCallback_(const td_navigation::Td_navigation_Status::ConstPtr& msg);
	bool controlCallback_(messages::MissionPlanningControl::Request &req, messages::MissionPlanningControl::Response &res);
	void queueEmptyTimerCallback_(const ros::TimerEvent &event);
};

#endif // MISSION_PLANNING_H

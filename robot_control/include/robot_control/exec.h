#ifndef EXEC_H
#define EXEC_H
#include <ros/ros.h>
#include <deque>
#include "action_type_enum.h"
#include "action.h"
#include "action_params.h"
#include "idle.h"
#include "drive_global.h"
#include "drive_relative.h"
#include "drive_to_wall.h"
#include "dig.h"
#include "dump.h"
#include "wait.h"
#include "raise_arm.h"
#include "partially_raise_arm.h"
#include "raise_bucket.h"
#include "lower_bucket.h"
#include <messages/ExecAction.h>
#include <messages/ActuatorOut.h>
#include <messages/ExecInfo.h>
#include <messages/ExecActionEnded.h>
#include <messages/NavFilterOut.h>
#include <messages/NextWaypointOut.h>
#include <messages/ExecManualOverride.h>
#include <robot_control/DriveSpeeds.h>
#include <hw_interface_plugin_roboteq/Roboteq_Data.h>

#define ACTION_POOL_SIZE 100

class Exec : public RobotControlInterface
{
public:
	// Members
	ros::NodeHandle nh;
	ros::Publisher infoPub;
	ros::Publisher actuatorPub;
	ros::Publisher actionEndedPub;
	ros::ServiceServer actionServ;
	ros::ServiceServer manualOverrideServ;
	ros::Subscriber navSub;
    ros::Subscriber scoopSub;
    ros::Subscriber armSub;
    ros::Subscriber bucketSub;
	ros::Subscriber driveSpeedsSub;
    ros::Subscriber leftDriveSub;
    ros::Subscriber rightDriveSub;

	const int loopRate = 20; // Hz
	// Methods
	Exec(); // Constructor
	void run(); // Main run method for exec
private:
	// Members
	std::deque <Action*> actionDeque_;
	Idle pauseIdle_;
	ACTION_TYPE_T nextActionType_ = _idle;
	bool newActionFlag_ = false;
	bool pushToFrontFlag_ = false;
	bool clearDequeFlag_ = false;
	bool clearFrontFlag_ = false;
	bool pause_ = true;
	bool pausePrev_ = true;
	bool manualOverride_ = false;
	bool actionDequeEmptyPrev_;
	int currentActionDone_ = 0;
	size_t actionDequeSize_ = 0;
	unsigned int actionPoolIndex_[NUM_ACTIONS];
	Action* actionPool_[NUM_ACTIONS][ACTION_POOL_SIZE];
	ACTION_PARAMS_T params_;
	messages::ActuatorOut actuatorMsgOut_;
	messages::ExecInfo execInfoMsgOut_;
	messages::ExecActionEnded execActionEndedMsgOut_;
	double execStartTime_;
	double execElapsedTime_;
	bool poseUpdateInProgress = false;
	// Methods
	bool actionCallback_(messages::ExecAction::Request &req, messages::ExecAction::Response &res);
	bool manualOverrideCallback_(messages::ExecManualOverride::Request &req, messages::ExecManualOverride::Response &res);
	void navCallback_(const messages::NavFilterOut::ConstPtr& msg);
	void scoopCallback_(const hw_interface_plugin_roboteq::Roboteq_Data::ConstPtr& msg);
	void armCallback_(const hw_interface_plugin_roboteq::Roboteq_Data::ConstPtr& msg);
	void bucketCallback_(const hw_interface_plugin_roboteq::Roboteq_Data::ConstPtr& msg);
	void driveSpeedsCallback_(const robot_control::DriveSpeeds::ConstPtr& msg);
    void leftDriveCallback_(const hw_interface_plugin_roboteq::Roboteq_Data::ConstPtr& msg);
    void rightDriveCallback_(const hw_interface_plugin_roboteq::Roboteq_Data::ConstPtr& msg);
	void packActuatorMsgOut_();
	void packInfoMsgOut_();
};

#endif // EXEC_H

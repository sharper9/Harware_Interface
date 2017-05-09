#ifndef MISSION_PLANNING_PROCESS_SHARE_H
#define MISSION_PLANNING_PROCESS_SHARE_H
#include <vector>
#include <ros/ros.h>
#include <robot_control/DriveSpeeds.h>
#include <robot_control/Waypoint.h>
#include <robot_control/cataglyphis_timer.h>
#include "robot_status.h"
#include "action_type_enum.h"
#include <messages/ExecAction.h>
#include <messages/ExecInfo.h>
#include <math.h>
#include <time.h>
#include "mission_planning_types_defines.h"
#include "planning_map_struct.hpp"
#include "irl_grid_map.hpp"

#define DIG_MAP_RES 1.0
#define DIG_MAP_X_LEN 7.38
#define DIG_MAP_Y_LEN 3.88

class MissionPlanningProcedureShare
{
public:
    static bool procsToExecute[NUM_PROC_TYPES];
    static bool procsToInterrupt[NUM_PROC_TYPES];
    static bool procsBeingExecuted[NUM_PROC_TYPES];
    static bool procsToResume[NUM_PROC_TYPES];
    static unsigned int numProcsBeingOrToBeExecOrRes;
    static unsigned int numProcsBeingOrToBeExec;
    static unsigned int numProcsToBeExecAndNotInterrupt;
    static ros::ServiceClient execActionClient;
    static messages::ExecAction execActionSrv;
    static ros::Subscriber execInfoSub;
    static messages::ExecInfo execInfoMsg;
    static ros::Publisher driveSpeedsPub;
    static robot_control::DriveSpeeds driveSpeedsMsg;
    static robot_control::DriveSpeeds driveSpeedsMsgPrev;
    static RobotStatus robotStatus;
    static CataglyphisTimerBase* timers[NUM_TIMERS];
    static std::vector<robot_control::Waypoint> waypointsToTravel;
    static int numWaypointsToTravel;
    static bool execDequeEmpty;
    static PROC_TYPES_T execLastProcType;
    static unsigned int execLastSerialNum;
    static bool recoverCondition;
    static bool queueEmptyTimedOut;
    static float distanceToDrive; // m
    static float angleToTurn; // deg
    static double missionTime;
    static double prevTime;
    static bool missionStarted;
    static bool initialized;
    static bool atMineLocation;
    static bool bucketFull;
    static bool atDepositLocation;
    static bool confirmedAtDepositLocation;
    static bool stuck;
	static IRLGridMap<PlanningMapData> digPlanningMap;
    const float depositWaypointX = 3.0; // m
    const float depositWaypointY = 0.0; // m
    const float depositWaypointDistanceTolerance = 0.2; // m
    const float depositWaypointAngleTolerance = 5.0; // deg
    const float depositWaypointRecoverX = 6.0; // m
    const float depositWaypointRecoverY = 0.0; // m
    const float queueEmptyTimerPeriod = 30.0; // sec
    const float defaultVMax = 1.0; // m/s
    const float defaultRMax = 45.0; // deg/s
	const float mapYOffset = 1.94; // m
	const float miningRegionMinXDistance = 4.55; // m
	const float miningRegionTargetXDistance = 4.65; // m
	const float miningWallBufferDistance = 1.0; // m
    const int numDigsPerMine = 3;
};

bool MissionPlanningProcedureShare::procsToExecute[NUM_PROC_TYPES];
bool MissionPlanningProcedureShare::procsToInterrupt[NUM_PROC_TYPES];
bool MissionPlanningProcedureShare::procsBeingExecuted[NUM_PROC_TYPES];
bool MissionPlanningProcedureShare::procsToResume[NUM_PROC_TYPES];
unsigned int MissionPlanningProcedureShare::numProcsBeingOrToBeExecOrRes;
unsigned int MissionPlanningProcedureShare::numProcsBeingOrToBeExec;
unsigned int MissionPlanningProcedureShare::numProcsToBeExecAndNotInterrupt;
ros::ServiceClient MissionPlanningProcedureShare::execActionClient;
messages::ExecAction MissionPlanningProcedureShare::execActionSrv;
ros::Subscriber MissionPlanningProcedureShare::execInfoSub;
messages::ExecInfo MissionPlanningProcedureShare::execInfoMsg;
ros::Publisher MissionPlanningProcedureShare::driveSpeedsPub;
robot_control::DriveSpeeds MissionPlanningProcedureShare::driveSpeedsMsg;
robot_control::DriveSpeeds MissionPlanningProcedureShare::driveSpeedsMsgPrev;
RobotStatus MissionPlanningProcedureShare::robotStatus;
CataglyphisTimerBase* MissionPlanningProcedureShare::timers[NUM_TIMERS];
std::vector<robot_control::Waypoint> MissionPlanningProcedureShare::waypointsToTravel;
int MissionPlanningProcedureShare::numWaypointsToTravel;
bool MissionPlanningProcedureShare::execDequeEmpty;
PROC_TYPES_T MissionPlanningProcedureShare::execLastProcType;
unsigned int MissionPlanningProcedureShare::execLastSerialNum;
bool MissionPlanningProcedureShare::initialized;
bool MissionPlanningProcedureShare::atMineLocation;
bool MissionPlanningProcedureShare::bucketFull;
bool MissionPlanningProcedureShare::atDepositLocation;
bool MissionPlanningProcedureShare::confirmedAtDepositLocation;
bool MissionPlanningProcedureShare::stuck;
bool MissionPlanningProcedureShare::recoverCondition;
bool MissionPlanningProcedureShare::queueEmptyTimedOut;
float MissionPlanningProcedureShare::distanceToDrive; // m
float MissionPlanningProcedureShare::angleToTurn; // deg
double MissionPlanningProcedureShare::missionTime;
double MissionPlanningProcedureShare::prevTime;
bool MissionPlanningProcedureShare::missionStarted;
IRLGridMap<PlanningMapData> MissionPlanningProcedureShare::digPlanningMap(DIG_MAP_RES, DIG_MAP_X_LEN, DIG_MAP_Y_LEN);

#endif // MISSION_PLANNING_PROCESS_SHARE_H

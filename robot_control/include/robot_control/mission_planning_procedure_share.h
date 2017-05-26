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
#include "bit_utils.h"

#define DIG_MAP_RES 1.0
#define DIG_MAP_X_LEN 7.38
#define DIG_MAP_Y_LEN 3.78

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
    static bool recoverLockout;
    static bool flipBackLockout;
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
    static bool tippedOver;
	static IRLGridMap<PlanningMapData> digPlanningMap;
    static float depositWaypointDistanceTolerance;
    static float depositWaypointAngleTolerance;
    static bool performFullPoseUpdate;
    static bool moderateQualityInitPoseFound;
    static float moderateQualityInitX;
    static float moderateQualityInitY;
    static float moderateQualityInitHeading;
    static int initManeuverToPerform;
    static float prevXPos;
    static float prevYPos;
    static double prevPosUnchangedTime;
    static bool tooCloseToWall;
    static bool tooCloseToWallLockout;
    const float depositWaypointX = 1.5; // m
    const float depositWaypointY = 0.0; // m
    const float depositWaypointDistanceToleranceInit = 0.25; // m
    const float depositWaypointAngleToleranceInit = 5.0; // deg
    const float depositWaypointDistanceToleranceMax = 0.4; // m
    const float depositWaypointAngleToleranceMax = 10.0; // deg
    const float depositWaypointDistanceToleranceIncrement = 0.05; // m
    const float depositWaypointAngleToleranceIncrement = 1.0; // deg
    const float depositWaypointRecoverX = 2.0; // m
    const float depositWaypointRecoverY = 0.0; // m
    const float queueEmptyTimerPeriod = 15.0; // sec
    const float defaultVMax = 1.0; // m/s
    const float defaultRMax = 45.0; // deg/s
	const float mapYOffset = 1.94; // m
    const float miningRegionMinXDistance = 4.2; // m (4.55)
    const float miningRegionTargetXDistance = 4.45; // m (4.65)
    const float miningWallBufferDistanceX = 0.5; // m
    const float miningWallBufferDistanceY = 0.5; // m
    const float miningWallPlanningDistanceX = 0.5; // m
    const float miningWallPlanningDistanceY = 1.25; // m
    const float robotCenterToScoopLength = 1.0; // m
    const float baseStationDistance = 1.776; // m
    const float maxStuckDistance = 0.75; // m
    const double maxStuckTime = 4.0; // sec
    const float flipBackDistanceToDrive = 0.3; // m
    const float tippedOverMaxPitchAngle = 20.0; // deg
    const float digWaypointDistanceTolerance = 0.5; // m
    const float digWaypointAngleTolerance = 15.0; // deg
    const int numDigsPerMine = 5;
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
bool MissionPlanningProcedureShare::tippedOver;
bool MissionPlanningProcedureShare::recoverLockout;
bool MissionPlanningProcedureShare::flipBackLockout;
bool MissionPlanningProcedureShare::queueEmptyTimedOut;
float MissionPlanningProcedureShare::distanceToDrive; // m
float MissionPlanningProcedureShare::angleToTurn; // deg
double MissionPlanningProcedureShare::missionTime;
double MissionPlanningProcedureShare::prevTime;
bool MissionPlanningProcedureShare::missionStarted;
IRLGridMap<PlanningMapData> MissionPlanningProcedureShare::digPlanningMap(DIG_MAP_RES, DIG_MAP_X_LEN, DIG_MAP_Y_LEN);
float MissionPlanningProcedureShare::depositWaypointDistanceTolerance;
float MissionPlanningProcedureShare::depositWaypointAngleTolerance;
bool MissionPlanningProcedureShare::performFullPoseUpdate;
bool MissionPlanningProcedureShare::moderateQualityInitPoseFound;
float MissionPlanningProcedureShare::moderateQualityInitX;
float MissionPlanningProcedureShare::moderateQualityInitY;
float MissionPlanningProcedureShare::moderateQualityInitHeading;
int MissionPlanningProcedureShare::initManeuverToPerform;
float MissionPlanningProcedureShare::prevXPos;
float MissionPlanningProcedureShare::prevYPos;
double MissionPlanningProcedureShare::prevPosUnchangedTime;
bool MissionPlanningProcedureShare::tooCloseToWall;
bool MissionPlanningProcedureShare::tooCloseToWallLockout;

#endif // MISSION_PLANNING_PROCESS_SHARE_H

#ifndef ROBOT_CONTROL_INTERFACE_H
#define ROBOT_CONTROL_INTERFACE_H
#include "robot_outputs.h"
#include "robot_status.h"
#include <ros/ros.h>
#include "bit_utils.h"

class RobotControlInterface
{
public:
    static RobotStatus robotStatus;
    static RobotOutputs robotOutputs;
	static Leading_Edge_Latch scoopStatusLEL;
	static Leading_Edge_Latch buckettatusLEL;
	static bool scoopEnded;
	static bool bucketEnded;
	static bool scoopFailed;
	static bool bucketFailed;
	const int scoopTol = 30;
	const int bucketTol = 50;
};

RobotStatus RobotControlInterface::robotStatus;
RobotOutputs RobotControlInterface::robotOutputs;
Leading_Edge_Latch RobotControlInterface::scoopStatusLEL;
Leading_Edge_Latch RobotControlInterface::buckettatusLEL;
bool RobotControlInterface::scoopEnded;
bool RobotControlInterface::bucketEnded;
bool RobotControlInterface::scoopFailed;
bool RobotControlInterface::bucketFailed;

#endif // ROBOT_CONTROL_INTERFACE_H

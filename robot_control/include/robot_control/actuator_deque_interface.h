#ifndef ACTUATOR_DEQUE_INTERFACE_H
#define ACTUATOR_DEQUE_INTERFACE_H
#include <deque>
#include "task.h"
#include "task_type_enum.h"
#include "drive_halt.h"
#include "drive_straight.h"
#include "drive_pivot.h"
//#include "drive_arc.h"
#include "scoop_halt.h"
#include "scoop_set_pos.h"
#include "arm_halt.h"
#include "arm_set_pos.h"
#include "bucket_halt.h"
#include "bucket_set_pos.h"
#define TASK_POOL_SIZE 100

class ActuatorDequeInterface
{
public:
	static std::deque <Task*> driveDeque;
	static std::deque <Task*> scoopDeque;
    static std::deque <Task*> armDeque;
	static std::deque <Task*> bucketDeque;
	static unsigned int taskPoolIndex[NUM_TASKS];
	static Task* taskPool[NUM_TASKS][TASK_POOL_SIZE];
	static DriveHalt driveHalt;
	static ScoopHalt scoopHalt;
    static ArmHalt armHalt;
	static BucketHalt bucketHalt;
	static int driveDequeEnded;
	static int driveDequeEmpty;
	static int driveDequeEmptyPrev;
	static int scoopDequeEnded;
	static int scoopDequeEmpty;
	static int scoopDequeEmptyPrev;
    static int armDequeEnded;
    static int armDequeEmpty;
    static int armDequeEmptyPrev;
	static int bucketDequeEnded;
	static int bucketDequeEmpty;
	static int bucketDequeEmptyPrev;
};

std::deque <Task*> ActuatorDequeInterface::driveDeque;
std::deque <Task*> ActuatorDequeInterface::scoopDeque;
std::deque <Task*> ActuatorDequeInterface::armDeque;
std::deque <Task*> ActuatorDequeInterface::bucketDeque;
unsigned int ActuatorDequeInterface::taskPoolIndex[NUM_TASKS] = {0};
Task* ActuatorDequeInterface::taskPool[NUM_TASKS][TASK_POOL_SIZE] = {0};
DriveHalt ActuatorDequeInterface::driveHalt;
ScoopHalt ActuatorDequeInterface::scoopHalt;
ArmHalt ActuatorDequeInterface::armHalt;
BucketHalt ActuatorDequeInterface::bucketHalt;
int ActuatorDequeInterface::driveDequeEnded;
int ActuatorDequeInterface::driveDequeEmpty;
int ActuatorDequeInterface::driveDequeEmptyPrev = 1;
int ActuatorDequeInterface::scoopDequeEnded;
int ActuatorDequeInterface::scoopDequeEmpty;
int ActuatorDequeInterface::scoopDequeEmptyPrev = 1;
int ActuatorDequeInterface::armDequeEnded;
int ActuatorDequeInterface::armDequeEmpty;
int ActuatorDequeInterface::armDequeEmptyPrev = 1;
int ActuatorDequeInterface::bucketDequeEnded;
int ActuatorDequeInterface::bucketDequeEmpty;
int ActuatorDequeInterface::bucketDequeEmptyPrev = 1;
#endif // ACTUATOR_DEQUE_INTERFACE_H

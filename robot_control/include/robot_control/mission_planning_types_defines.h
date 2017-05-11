#ifndef MISSION_PLANNING_TYPES_DEFINES_H
#define MISSION_PLANNING_TYPES_DEFINES_H

#define PI 3.14159265359
#define DEG2RAD PI/180.0
#define RAD2DEG 180.0/PI
#define NUM_PROC_TYPES 7
#define NUM_TIMERS 1
// !!! If PROC_TYPES_T is ever edited, edit controlCallback_ in MissionPlanning as well
enum PROC_TYPES_T {__initialize__, __driveToDig__, __mine__, __driveToDeposit__, __depositRealign__, __deposit__, __recover__};
enum TIMER_NAMES_T {_queueEmptyTimer_};
enum PROC_STATE_T {_init_, _exec_, _interrupt_, _finish_};

#endif //MISSION_PLANNING_TYPES_DEFINES_H


#ifndef TASK_TYPE_ENUM_H
#define TASK_TYPE_ENUM_H

#define NUM_TASKS 9 // 10
enum TASK_TYPE_T {_driveHalt_, _driveStraight_, _pivot_, /*_driveArc_,*/ _scoopHalt_, _scoopSetPos_, _armHalt_, _armSetPos_, _bucketHalt_, _bucketSetPos_};
#endif // TASK_TYPE_ENUM_H

// !!!! When _driveArc_ is added, do not forget to uncomment it in action.cpp, line 5

#ifndef DRIVE_TO_WALL_H
#define DRIVE_TO_WALL_H
#include "action.h"

class DriveToWall : public Action
{
public:
	void init();
	int run();
private:
    float vMax_;
    float rMax_;
    bool pushedToFront_;
    bool driveBackwards_;
    bool driveCompleted_;
};

#endif // DRIVE_TO_WALL_H

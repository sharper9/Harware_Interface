#ifndef CLOSE_TO_WALL_H
#define CLOSE_TO_WALL_H
#include "procedure.h"

class CloseToWall : public Procedure
{
public:
    // Methods
    bool runProc();
    // Members
    enum CLOSE_TO_WALL_STAGE {_fullPose, _driveToLocation} stage;

};

#endif // CLOSE_TO_WALL_H

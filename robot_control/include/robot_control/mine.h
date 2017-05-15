#ifndef MINE_H
#define MINE_H
#include "procedure.h"

class Mine : public Procedure
{
public:
    // Methods
    bool runProc();
    // Members
    Leading_Edge_Latch tooCloseToWallLatch;
    bool tooCloseToWall;
    float backUpDistance = 1.5; // m
};

#endif // MINE_H

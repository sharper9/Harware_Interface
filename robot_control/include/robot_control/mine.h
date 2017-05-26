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
    float backUpDistance = 0.5; // m
    unsigned int finalSerialNum;
    float digPitchAngle;
};

#endif // MINE_H

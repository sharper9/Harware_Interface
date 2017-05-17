#ifndef INITIALIZE_H
#define INITIALIZE_H
#include "procedure.h"

class Initialize : public Procedure
{
public:
    // Methods
    bool runProc();
    // Members
    double startupTime;
    enum INITIALIZE_STAGE_T {_startTimer, _checkFullPose, _initRaiseBucket, _drive, _initialRaiseArm} stage;
    bool initComplete;
    const double waitForFullPoseTime = 5.0; // sec
    float rotateDeltaAngle; // deg
};

#endif // INITIALIZE_H

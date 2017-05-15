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
    enum INITIALIZE_STAGE_T {_initialRaiseArm, _startTimer, _checkFullPose, _rotate} stage;
    bool initComplete;
    const double waitForFullPoseTime = 5.0; // sec
    const float rotateDeltaAngle = 25.0; // deg
};

#endif // INITIALIZE_H

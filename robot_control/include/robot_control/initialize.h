#ifndef INITIALIZE_H
#define INITIALIZE_H
#include "procedure.h"

class Initialize : public Procedure
{
public:
    // Methods
    bool runProc();
    bool knownPositionCheckManeuver();
    // Members
    double startupTime;
    enum INITIALIZE_STAGE_T {_startTimer, _checkFullPose, _moveActuator, _completeInit} stage;
    INITIALIZE_STAGE_T nextStage;
    bool initComplete;
    const double waitForFullPoseTime = 5.0; // sec
    float driveDeltaDistance;
    float rotateDeltaAngle; // deg
    bool bucketRaised;
    struct INITIALIZE_MANEUVER_T {float driveDistance; float turnAngle;};
    std::vector<INITIALIZE_MANEUVER_T> knownPoseManeuvers;
    std::vector<INITIALIZE_MANEUVER_T> unknownPoseManeuvers;
    float xPosToUse;
    float yPosToUse;
    float headingToUse;
    bool performAManeuver;
    bool raiseScoopFullyBeforeManeuver;
};

#endif // INITIALIZE_H

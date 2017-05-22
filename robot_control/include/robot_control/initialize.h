#ifndef INITIALIZE_H
#define INITIALIZE_H
#include "procedure.h"

#define NUM_INIT_ACTIONS 15

class Initialize : public Procedure
{
public:
    // Methods
    Initialize();
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
    std::vector<INITIALIZE_MANEUVER_T> unknownPoseManeuvers;
    float xPosToUse;
    float yPosToUse;
    float headingToUse;
    bool performAManeuver;
    bool raiseScoopFullyBeforeManeuver;

    bool armRaised;
    enum INIT_ACTION_TYPE_T {__straight, __turn, __armRaise, __bucketRaisePartially, __bucketLower};
    class InitAction
    {
    public:
        InitAction(INIT_ACTION_TYPE_T actionTypeIn);
        InitAction(INIT_ACTION_TYPE_T actionTypeIn, float parameterIn);
        void pushAction();
        float parameter;
        INIT_ACTION_TYPE_T actionType;
    };
    std::vector<std::vector<InitAction>> actionList;
};

#endif // INITIALIZE_H

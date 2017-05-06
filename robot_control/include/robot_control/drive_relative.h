#ifndef DRIVE_RELATIVE_H
#define DRIVE_RELATIVE_H
#include "action.h"

class DriveRelative : public Action
{
public:
	void init();
	int run();
private:
	enum DRIVE_RELATIVE_STEP_T {_computeManeuver, _performManeuver} step_;
    float desiredX_;
    float desiredY_;
    float vMax_;
    float rMax_;
    float desiredEndHeading_;
    bool endHeading_;
    float candidateEndHeadingAngleToTurn_[2];
    float distanceToDrive_;
    float angleToTurn_;
    float xErr_;
    float yErr_;
    float uXDes_;
    float uYDes_;
    float uXAct_;
    float uYAct_;
    float newHeadingSign_;
    bool pushedToFront_;
    bool driveBackwards_;
    bool driveCompleted_;
    void calculatePath_();
};

#endif // DRIVE_RELATIVE_H

#ifndef ARM_LOWER_TO_PITCH_H
#define ARM_LOWER_TO_PITCH_H
#include "task.h"

class ArmLowerToPitch : public Task
{
public:
	void init();
	int run();
private:
	int returnValue_;
	float initialPitch_;
	double initialTime_;
	const int armPosDelta_ = -16;
    float deltaPitchGoal_ = -2.0; // deg
	const double timeoutValue_ = 4.0; // sec
};

#endif // ARM_LOWER_TO_PITCH_H

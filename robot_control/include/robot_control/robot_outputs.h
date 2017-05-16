#ifndef ROBOT_OUTPUTS_H
#define ROBOT_OUTPUTS_H
#include <stdint.h>

#define SCOOP_RAISED -1000
#define SCOOP_LOWERED 500
#define ARM_RAISED 1000
#define ARM_LOWERED -1000
#define ARM_INIT -900
//#define ARM_LOWERED -100 // TODO: This is temporary for testing indoors
#define ARM_DUMP 0
#define BUCKET_RAISED 1000
#define BUCKET_LOWERED -1000

class RobotOutputs
{
public:
    // Drive outputs; Range: [-1000,1000] positive is forward, negative is reverse
	int16_t flMotorSpeed = 0;
	int16_t blMotorSpeed = 0;
	int16_t frMotorSpeed = 0;
	int16_t brMotorSpeed = 0;
	bool stopFlag = true;
	bool turnFlag = false;
	// Scoop output
	int16_t scoopPosCmd = SCOOP_RAISED;
	uint8_t scoopStopCmd = 0;
    // Arm output
    int16_t armPosCmd = ARM_INIT;
    uint8_t armStopCmd = 0;
	// Bucket output
	int16_t bucketPosCmd = BUCKET_LOWERED;
	uint8_t bucketStopCmd = 0;
};

#endif // ROBOT_OUTPUTS_H

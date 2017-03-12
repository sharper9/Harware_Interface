#include <robot_control/arm_halt.h>

void ArmHalt::init()
{
    robotOutputs.armStopCmd = 1;
}

int ArmHalt::run()
{
    robotOutputs.armStopCmd = 1;
	return 1;
}

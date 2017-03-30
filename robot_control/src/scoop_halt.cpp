#include <robot_control/scoop_halt.h>

void ScoopHalt::init()
{
    robotOutputs.scoopStopCmd = 1;
}

int ScoopHalt::run()
{
    robotOutputs.scoopStopCmd = 1;
	return 1;
}

#ifndef ARM_HALT_H
#define ARM_HALT_H
#include "task.h"

class ArmHalt : public Task
{
public:
	void init();
	int run();
};

#endif // ARM_HALT_H

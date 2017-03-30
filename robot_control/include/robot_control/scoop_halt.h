#ifndef SCOOP_HALT_H
#define SCOOP_HALT_H
#include "task.h"

class ScoopHalt : public Task
{
public:
	void init();
	int run();
};

#endif // SCOOP_HALT_H

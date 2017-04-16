#ifndef ARM_SET_POS_H
#define ARM_SET_POS_H
#include "task.h"

class ArmSetPos : public Task
{
public:
	void init();
	int run();
private:
    int armPos_;
	int returnValue_;
};

#endif // ARM_SET_POS_H

#ifndef SCOOP_SET_POS_H
#define SCOOP_SET_POS_H
#include "task.h"

class ScoopSetPos : public Task
{
public:
	void init();
	int run();
private:
    int scoopPos_;
	int returnValue_;
};

#endif // SCOOP_SET_POS_H

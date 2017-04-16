#ifndef BUCKET_HALT_H
#define BUCKET_HALT_H
#include "task.h"

class BucketHalt : public Task
{
public:
	void init();
	int run();
};

#endif // BUCKET_HALT_H

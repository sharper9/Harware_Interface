#ifndef BUCKET_SET_POS_H
#define BUCKET_SET_POS_H
#include "task.h"

class BucketSetPos : public Task
{
public:
	void init();
	int run();
private:
    int bucketPos_;
	int returnValue_;
};

#endif // BUCKET_SET_POS_H

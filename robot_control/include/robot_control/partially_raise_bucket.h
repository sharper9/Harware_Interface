#ifndef PARTIALLY_RAISE_BUCKET_H
#define PARTIALLY_RAISE_BUCKET_H
#include "action.h"

class PartiallyRaiseBucket : public Action
{
public:
    void init();
    int run();
};

#endif // PARTIALLY_RAISE_BUCKET_H

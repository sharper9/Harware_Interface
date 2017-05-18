#ifndef RAISE_BUCKET_H
#define RAISE_BUCKET_H
#include "action.h"

class RaiseBucket : public Action
{
public:
    void init();
    int run();
};

#endif // RAISE_BUCKET_H

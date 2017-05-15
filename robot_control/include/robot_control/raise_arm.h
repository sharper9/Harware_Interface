#ifndef RAISE_ARM_H
#define RAISE_ARM_H
#include "action.h"

class RaiseArm : public Action
{
public:
    void init();
    int run();
};

#endif // RAISE_ARM_H

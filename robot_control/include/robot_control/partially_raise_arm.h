#ifndef PARTIALLY_RAISE_ARM_H
#define PARTIALLY_RAISE_ARM_H
#include "action.h"

class PartiallyRaiseArm : public Action
{
public:
    void init();
    int run();
};

#endif // PARTIALLY_RAISE_ARM_H

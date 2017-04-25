#ifndef DIG_H
#define DIG_H
#include "action.h"

class Dig : public Action
{
public:
    void init();
    int run();
};

#endif // DIG_H

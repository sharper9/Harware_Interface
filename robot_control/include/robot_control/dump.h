#ifndef DUMP_H
#define DUMP_H
#include "action.h"

class Dump : public Action
{
public:
    void init();
    int run();
};

#endif // DUMP_H

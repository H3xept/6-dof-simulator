#ifndef __SIMULATION_H__
#define __SIMULATION_H__

#include <list>
#include "Environment.h"
#include "Logger.h"

class Simulation {
    virtual void start() = 0;
    virtual void pause() = 0;
    virtual void resume() = 0;
};

#endif // __SIMULATION_H__
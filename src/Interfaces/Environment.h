#ifndef __ENVIRONMENT_H__
#define __ENVIRONMENT_H__

#include <list>
#include "EnvironmentObject.h"
#include "Logger.h"

class Environment : public TimeHandler {
public:
    virtual ~Environment() {}
    Logger* logger;
    std::list<EnvironmentObject*> env_objects;
};

#endif // __ENVIRONMENT_H__
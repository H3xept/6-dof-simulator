#ifndef __FLATEARTH_H__
#define __FLATEARTH_H__

#include "Interfaces/Environment.h"
#include "Interfaces/Logger.h"
#include "Interfaces/PrettyPrintable.h"

class FlatEarth : public Environment, public PrettyPrintable {
public: 
    FlatEarth(Logger& logger);
    ~FlatEarth();
    
    void update(double dt) override;
    std::string str() override;
};

#endif // __FLATEARTH_H__
#ifndef __THRUSTFIXEDWING_EXTENSION_H__
#define __THRUSTFIXEDWING_EXTENSION_H__

#include <ForceModels/ThrustFixedWing.h>
#include "../Containers/DroneConfig.h"

class ThrustFixedWing : public caelus_fdm::ThrustFixedWing {
public:
    ThrustFixedWing(DroneConfig conf) : caelus_fdm::ThrustFixedWing(
        conf.b_prop,
        NULL
    ) {};
}; 

#endif // __THRUSTFIXEDWING_EXTENSION_H__
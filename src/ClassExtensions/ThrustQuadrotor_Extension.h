#ifndef __THRUSTQUADROTOR_EXTENSION_H__
#define __THRUSTQUADROTOR_EXTENSION_H__

#include <ForceModels/ThrustQuadrotor.h>
#include "../Containers/DroneConfig.h"

class ThrustQuadrotor : public caelus_fdm::ThrustQuadrotor {
public:
    ThrustQuadrotor(DroneConfig conf) : caelus_fdm::ThrustQuadrotor(
        conf.b,
        conf.d,
        conf.l,
        NULL
    ) {};
};

#endif // __THRUSTQUADROTOR_EXTENSION_H__
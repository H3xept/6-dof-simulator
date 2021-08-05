#ifndef __FIXEDWINGEOM_EXTENSION_H__
#define __FIXEDWINGEOM_EXTENSION_H__

#include <EquationsOfMotion/FixedWingEOM.h>
#include "../Containers/DroneConfig.h"

class FixedWingEOM : public caelus_fdm::FixedWingEOM {
public:
    FixedWingEOM(DroneConfig conf) : caelus_fdm::FixedWingEOM(
        conf.b_prop,
        conf.c,
        conf.b_aero,
        conf.S,
        conf.drone_aero_config,
        conf.J,
        NULL,
        conf.mass,
        9.81
    ) {}
};

#endif // __FIXEDWINGEOM_EXTENSION_H__
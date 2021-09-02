#ifndef __QUADROTOREOM_EXTENSION_H__
#define __QUADROTOREOM_EXTENSION_H__

#include <EquationsOfMotion/MixedEOM.h>
#include "../Containers/DroneConfig.h"

class MixedEOM : public caelus_fdm::MixedEOM {
public:
    MixedEOM(DroneConfig conf) : caelus_fdm::MixedEOM(
        conf.b_prop,
        conf.c,
        conf.b_aero,
        conf.S,
        conf.d,
        conf.l,
        conf.drone_aero_config,
        conf.J,
        NULL,
        NULL,
        conf.mass,
        9.81
    ) {}

    bool get_airborne_status() {
        return this->is_airborne;
    }
};

#endif // __MIXEDEOM_EXTENSION_H__
#ifndef __WEIGHTFM_EXTENSION_H__
#define __WEIGHTFM_EXTENSION_H__

#include <ForceModels/Weight.h>
#include "../Containers/DroneConfig.h"

class Weight : public caelus_fdm::Weight {
public:
    Weight(DroneConfig conf, double g_force) : caelus_fdm::Weight(
        conf.mass,
        g_force
    ) {}
};

#endif // __WEIGHTFM_EXTENSION_H__
#ifndef __PROPELLER_H__
#define __PROPELLER_H__

#include <Eigen/Eigen>
#include "Interfaces/AsyncDroneControl.h"

#define VTOL_PROPELLERS_N 4
#define THRUST_PROPELLERS_N 1
#define PROP_N (THRUST_PROPELLERS_N)

class Propellers : public AsyncDroneControl {
private:
    Eigen::VectorXd _last_control{PROP_N};
public:
    Propellers();
    // Clockwise [0:4] VTOL propeller
    // [4] Back-Thrust propeller
    Eigen::VectorXd control(double dt) override;
    void set_control(Eigen::VectorXd c) override;
};

#endif // __PROPELLER_H__
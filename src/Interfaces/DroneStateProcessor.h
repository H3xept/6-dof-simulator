#ifndef __DRONESTATEPROCESSOR_H__
#define __DRONESTATEPROCESSOR_H__

#include <Eigen/Eigen>

class DroneStateProcessor {
public:
    virtual void new_drone_state(Eigen::VectorXd state, Eigen::VectorXd dx_state) = 0;
};

#endif // __DRONESTATEPROCESSOR_H__
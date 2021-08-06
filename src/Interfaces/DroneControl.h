#ifndef __DRONECONTROLS_H__
#define __DRONECONTROLS_H__

#include <Eigen/Eigen>

class DroneControl {
public:
    ~DroneControl() {}
    virtual Eigen::VectorXd get_control(double dt) = 0;
};  

#endif // __DRONECONTROLS_H__
#ifndef __ASYNCDRONECONTROL_H__
#define __ASYNCDRONECONTROL_H__

#include <Eigen/Eigen>
#include "DroneControl.h"

class AsyncDroneControl : public DroneControl {
public:
    virtual Eigen::VectorXd last_control() = 0;
    virtual void set_control(Eigen::VectorXd c) = 0;
}

#endif // __ASYNCDRONECONTROL_H__
#ifndef __AILERON_H__
#define __AILERON_H__

#include <Eigen/Eigen>
#include "Interfaces/AsyncDroneControl.h"

class Ailerons : public AsyncDroneControl {
private:
    Eigen::VectorXd _last_control;
public:
    Ailerons(uint8_t ailerons_n);
    // [0] Left
    // [1] Right
    Eigen::VectorXd control(double dt) override;
    void set_control(Eigen::VectorXd c) override;
};


#endif // __AILERON_H__
#include "Propellers.h"

Eigen::VectorXd Propellers::control(double dt) {
    return this->_last_control;
}

void Propellers::set_control(Eigen::VectorXd c) {
    this->_last_control = c;
}

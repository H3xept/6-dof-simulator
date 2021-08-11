#include "Ailerons.h"

Eigen::VectorXd Ailerons::control(double dt) {
    return this->_last_control;
}

void Ailerons::set_control(Eigen::VectorXd c) {
    this->_last_control = c;
}

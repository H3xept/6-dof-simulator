#include "Propellers.h"

Propellers::Propellers() {
    for (int i = 0; i < PROP_N; i++) this->_last_control[i] = 0;
}

Eigen::VectorXd Propellers::control(double dt) {
    return this->_last_control;
}

void Propellers::set_control(Eigen::VectorXd c) {
    this->_last_control = c;
}

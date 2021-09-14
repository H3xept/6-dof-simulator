#include "Ailerons.h"

Ailerons::Ailerons(uint8_t ailerons_n) : _last_control(ailerons_n) {
    for (int i = 0; i < ailerons_n; i++) this->_last_control[i] = 0;
}

Eigen::VectorXd Ailerons::control(double dt) {
    return this->_last_control;
}

void Ailerons::set_control(Eigen::VectorXd c) {
    this->_last_control = c;
}

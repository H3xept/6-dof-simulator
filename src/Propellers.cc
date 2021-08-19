#include "Propellers.h"
#include <math.h>

Propellers::Propellers() {
    for (int i = 0; i < PROP_N; i++) this->_last_control[i] = 0;
}

Eigen::VectorXd Propellers::control(double dt) {
    // Using Erik's formula for omega here
    // Motor parameters -- temporary! Move to somewhere more proper.
    double Kt = 6.2e-05; // thrust [N] 
    double Km = Kt / 42; // torque [Nm]
    double Me = 1 / (490 * 3.14159265 / 30); // back EMF constant [V / [rad/s]]
    double Mt = Me; // torque constant [Nm/Amp]

    double Ly = 0.4; 
    double Lx = 1.09;
    double Lxf = 0.456;
    double Lxr = Lx - Lxf;

    double Scell = 7;
    double Rs = 0.10;
    double lm = 0.0007;
    
    double lxx = 1.0;
    double lxy = 0.05;
    double lxz = 0.05;
    double lyx = 0.05;
    double lyy = 1.5;
    double lyz = 0.05;
    double lzx = 0.05;
    double lzy = 0.05;
    double lzz = 2;

    double Mcg = 14.5;
    double battery_voltage = 4;
    double u = abs(this->_last_control[0]) * battery_voltage;

    Eigen::VectorXd ret{1};
    ret[0] = ((-Mt * Me / Rs) + sqrt(pow((Mt * Me / Rs), 2) - 4 * Km * -Mt / Rs * u)) / 2 * Km;

    return ret;
}

void Propellers::set_control(Eigen::VectorXd c) {
    this->_last_control = c;
}

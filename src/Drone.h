#ifndef __DRONE_H__
#define __DRONE_H__

#include <Eigen/Eigen>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <boost/numeric/odeint.hpp>
#include <EquationsOfMotion/FixedWingEOM.h>
#include "Containers/DroneConfig.h"
#include "Interfaces/DynamicObject.h"
#include "ClassExtensions/FixedWingEOM_Extension.h"

#define STATE_SIZE 12

typedef boost::numeric::odeint::runge_kutta_dopri5<Eigen::VectorXd,double,Eigen::VectorXd,double,boost::numeric::odeint::vector_space_algebra> ODESolver;

class Drone : public DynamicObject {
private:
    DroneConfig config;
    Eigen::VectorXd state;
    FixedWingEOM dynamics;
    ODESolver dynamics_solver;
public:

    Drone(char* config_file);
    ~Drone() {};

    void update(double dt) override;

    Eigen::VectorXd& get_state() override {
        return this->state;
    }

    DroneConfig get_config() {
        return this->config;    
    }

};

#endif // __DRONE_H__
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
#include "Interfaces/MAVLinkSystem.h"
#include "Interfaces/Alive.h"
#include "ClassExtensions/FixedWingEOM_Extension.h"
#include "Interfaces/MAVLinkMessageRelay.h"

#define STATE_SIZE 12

typedef boost::numeric::odeint::runge_kutta_dopri5<Eigen::VectorXd,double,Eigen::VectorXd,double,boost::numeric::odeint::vector_space_algebra> ODESolver;

class Drone : public DynamicObject, public MAVLinkSystem {
private:
    DroneConfig config;
    Eigen::VectorXd state;
    FixedWingEOM dynamics;
    ODESolver dynamics_solver;
    MAVLinkMessageRelay& connection;
public:

    Drone(char* config_file, MAVLinkMessageRelay& connection);
    ~Drone() {};

    Eigen::VectorXd& get_state() override { return this->state; }
    DroneConfig get_config() { return this->config; }

    void update(double dt) override;
    MAVLinkMessageRelay& get_mavlink_message_relay() override;
};

#endif // __DRONE_H__
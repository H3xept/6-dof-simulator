#ifndef __DRONE_H__
#define __DRONE_H__

#include <Eigen/Eigen>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <EquationsOfMotion/FixedWingEOM.h>
#include "Containers/DroneConfig.h"
#include "Interfaces/DynamicObject.h"
#include "ClassExtensions/FixedWingEOM_Extension.h"

#define STATE_SIZE 12

class Drone : public DynamicObject {
private:
    DroneConfig config;
    Eigen::VectorXd state;
    FixedWingEOM dynamics;
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
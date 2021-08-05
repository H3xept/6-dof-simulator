#ifndef __DRONE_H__
#define __DRONE_H__

#include <Eigen/Eigen>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <EquationsOfMotion/FixedWingEOM.h>
#include "Interfaces/DynamicObject.h"
#include "Containers/DroneConfig.h"

class Drone : public DynamicObject {
private:
    DroneConfig config;
public:
    Drone(char* config_file);
    Drone(DroneConfig c);
    ~Drone() {};
    void update(double dt) override;
    
    DroneConfig get_config() {
        return this->config;    
    }

};

#endif // __DRONE_H__
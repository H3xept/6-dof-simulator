#include "Drone.h"

Drone::Drone(char* config_file) {
    std::ifstream fin(config_file);
    fin >> this->config;
}

Drone::Drone(DroneConfig c) : config(c) {}
void Drone::update(double dt) {}
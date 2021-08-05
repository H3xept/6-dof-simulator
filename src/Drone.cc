#include "Drone.h"

DroneConfig config_from_file_path(char* path) {
    DroneConfig conf;
    std::ifstream fin(path);
    fin >> conf;
    return conf;
}

Drone::Drone(char* config_file) : 
    state(STATE_SIZE),
    config(config_from_file_path(config_file)),
    dynamics(config) {}

void Drone::update(double dt) {}
#include "Drone.h"

DroneConfig config_from_file_path(char* path) {
    DroneConfig conf;
    std::ifstream fin(path);
    fin >> conf;
    return conf;
}

Drone::Drone(char* config_file, MAVLinkMessageRelay& connection) : 
    state(STATE_SIZE),
    config(config_from_file_path(config_file)),
    dynamics(config),
    dynamics_solver(),
    connection(connection),
    MAVLinkSystem::MAVLinkSystem(1, 200) {}

void Drone::update(double dt) {
    MAVLinkSystem::update(dt);
    // printf("Drone tick %f\n", dt);
}

MAVLinkMessageRelay& Drone::get_mavlink_message_relay() {
    return this->connection;
}
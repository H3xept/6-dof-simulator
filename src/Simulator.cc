#include "Simulator.h"
#include "Logging/ConsoleLogger.h"

Simulator::Simulator(SimulatorConfig c) : 
    config(c),
    logger(new ConsoleLogger())
{} 

Simulator::~Simulator() {}

SimulatorConfig Simulator::get_config() {
    return this->config;
}

void Simulator::start() {
    this->logger->log(SIMULATION_STARTED);
}

void Simulator::pause() {
    this->logger->log(SIMULATION_PAUSED);
}

void Simulator::resume() {
    this->logger->log(SIMULATION_RESUMED);
}

std::string Simulator::str() {
    SimulatorConfig c = this->get_config();
    return std::string(
        "<Simulator timestep: " + 
        std::to_string(c.timestep_ms) + 
        " | speed_mult: " +
        std::to_string(c.max_speed_multiplier) +
        " >"
    );
}

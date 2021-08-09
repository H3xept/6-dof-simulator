#include "Simulator.h"
#include "Logging/ConsoleLogger.h"

Simulator::Simulator(SimulatorConfig c) : 
    config(c),
    logger(new ConsoleLogger())
{
} 

Simulator::~Simulator() {}

SimulatorConfig Simulator::get_config() {
    return this->config;
}

void Simulator::add_environment_object(EnvironmentObject& e) {
    this->logger->log("Adding env object %s\n to %s\n" + e.str() +  " " + this->str());
    this->env_objects.push_back(&e);
}

void Simulator::update(double dt) {
    for (auto e : this->env_objects) {
        e->update(dt);
    }
}

void Simulator::start() {
    this->logger->log(SIMULATION_STARTED);
    while(true) {
        this->update(100);
    }
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

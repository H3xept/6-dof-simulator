#include <boost/format.hpp>
#include <boost/thread/thread.hpp>
#include "Simulator.h"
#include "Logging/ConsoleLogger.h"

Simulator::Simulator(SimulatorConfig c, MAVLinkMessageRelay& message_relay) : 
    config(c),
    message_relay(message_relay),
    logger(ConsoleLogger::shared_instance())
{
    // this->message_relay.add_message_handler(this);
    this->simulation_clock.set_timestep(this->config.timestep_us);
} 

Simulator::~Simulator() {}

SimulatorConfig Simulator::get_config() {
    return this->config;
}

void Simulator::add_environment_object(EnvironmentObject& e) {
    this->env_objects.push_back(&e);
}

void Simulator::update(boost::chrono::microseconds us) {
    this->simulation_clock.step();
    this->_process_mavlink_messages();
    for (auto e : this->env_objects) {
        e->update(us);
    }
}

void Simulator::handle_mavlink_message(mavlink_message_t m) {
    this->message_queue.push(m);
}

void Simulator::start() {
    this->logger->log(SIMULATION_STARTED);
    while(!this->should_shutdown) {
        if (this->config.running_lockstep) {
            boost::chrono::microseconds time_increment = this->get_config().timestep_us;
            this->update(time_increment);
            boost::this_thread::sleep_for(boost::chrono::microseconds(3000));
        } else {
            printf("NON-LOCKSTEP NOT SUPPORTED\n");
        }
    }
}

void Simulator::_process_mavlink_message(mavlink_message_t m) {
    this->should_advance_time = true;
}

void Simulator::_process_mavlink_messages() {
    this->message_queue.consume_all([this](mavlink_message_t m){
        this->_process_mavlink_message(m);
    });
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
        std::to_string(c.timestep_us.count()) + 
        " | speed_mult: " +
        std::to_string(c.max_speed_multiplier) +
        " >"
    );
}
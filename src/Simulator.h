#ifndef __SIMULATOR_H__
#define __SIMULATOR_H__

#include <boost/chrono.hpp>
#include <memory>
#include <boost/lockfree/queue.hpp>
#include "Interfaces/PrettyPrintable.h"
#include "Interfaces/Environment.h"
#include "Interfaces/MAVLinkMessageRelay.h"
#include "Interfaces/MAVLinkMessageHandler.h"
#include "Interfaces/Clock.h"

#define SIMULATION_STARTED "Simulation started."
#define SIMULATION_PAUSED "Simulation paused."
#define SIMULATION_RESUMED "Simulation resumed."

struct SimulatorConfig {
    boost::chrono::microseconds timestep_us;
    double max_speed_multiplier;
    bool running_lockstep;

    SimulatorConfig(long timestep_us, double max_speed_multiplier, bool running_lockstep) :
        timestep_us(timestep_us),
        max_speed_multiplier(max_speed_multiplier),
        running_lockstep(running_lockstep) {
            printf("Simulator has timestep of %ld (%lld)\n", timestep_us, this->timestep_us.count());
        };
};

class Simulator : 
    public Environment,
    public PrettyPrintable,
    public MAVLinkMessageHandler
    {
private:
    boost::chrono::microseconds stop_after_us{0};

    SimulatorConfig config;
    boost::lockfree::queue<mavlink_message_t, boost::lockfree::capacity<50>> message_queue;

    bool should_advance_time = false;
    bool should_shutdown = false;

    void _process_mavlink_message(mavlink_message_t m);
    void _process_mavlink_messages();
public:   
    Clock simulation_clock;
    std::unique_ptr<Logger> logger;
    
    Simulator(SimulatorConfig c);
    ~Simulator();

    SimulatorConfig get_config();

    std::string str() override;
    void handle_mavlink_message(mavlink_message_t m) override;

    void add_environment_object(EnvironmentObject& e) override;

    void update(boost::chrono::microseconds ms) override;
    void start() override;
    void pause() override;
    void resume() override;

    void start(boost::chrono::microseconds stop_after) {
        this->stop_after_us = stop_after;
        this->start();
    }
};

#endif // __SIMULATOR_H__
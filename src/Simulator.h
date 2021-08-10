#ifndef __SIMULATOR_H__
#define __SIMULATOR_H__

#include <memory>
#include "Interfaces/PrettyPrintable.h"
#include "Interfaces/Environment.h"

#define SIMULATION_STARTED "Simulation started."
#define SIMULATION_PAUSED "Simulation paused."
#define SIMULATION_RESUMED "Simulation resumed."

struct SimulatorConfig {
    double timestep_ms;
    double max_speed_multiplier;
};

class Simulator : public Environment, public PrettyPrintable {
private:
    SimulatorConfig config;
    std::chrono::steady_clock::time_point last_sim_time = std::chrono::steady_clock::now();
public:    
    std::unique_ptr<Logger> logger;
    
    Simulator(SimulatorConfig c);
    ~Simulator();

    SimulatorConfig get_config();

    std::string str() override;
    
    void add_environment_object(EnvironmentObject& e) override;

    void update(double dt) override;
    void start() override;
    void pause() override;
    void resume() override;
};

#endif // __SIMULATOR_H__
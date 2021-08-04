#ifndef __SIMULATOR_H__
#define __SIMULATOR_H__

#include "Interfaces/Simulation.h"
#include "Interfaces/PrettyPrintable.h"

#define SIMULATION_STARTED "Simulation started."
#define SIMULATION_PAUSED "Simulation paused."
#define SIMULATION_RESUMED "Simulation resumed."

struct SimulatorConfig {
    double timestep_ms;
    double max_speed_multiplier;
};

class Simulator : public Simulation, public PrettyPrintable {
private:
    SimulatorConfig config;
public:    
    std::unique_ptr<Environment> environment;
    std::unique_ptr<Logger> logger;
    
    Simulator(SimulatorConfig c);
    ~Simulator();

    SimulatorConfig get_config();

    std::string str() override;
    
    void start() override;
    void pause() override;
    void resume() override;
};

#endif // __SIMULATOR_H__
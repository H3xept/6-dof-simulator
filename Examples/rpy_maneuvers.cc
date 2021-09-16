
#include "../src/StandaloneDrone.h"
#include "../src/Simulator.h"
#include "../src/Controllers/SimpleFixedWingController.h"
#include "../src/Logging/DroneStateLogger.h"
#include <Eigen/Eigen>

ManoeuvrePlan* roll(const char* config) {

    std::vector<Manoeuvre> manoeuvres{Manoeuvre::HOLD, Manoeuvre::ROLL};
    std::vector<boost::chrono::microseconds> section_lenghts{
        boost::chrono::microseconds{4000}, // 4ms
        boost::chrono::microseconds{4000 * 250}, // 12 ms (3 timesteps)
    };
    return new ManoeuvrePlan{section_lenghts, manoeuvres};
}

int main()
{
    long time_step_us{4000};
    const char* fixed_wing_config = "../drone_models/fixed_wing";
    DroneStateLogger dsLog;
    SimpleFixedWingController quadController{config_from_file_path(fixed_wing_config)};
    
    ManoeuvrePlan* plan = roll(fixed_wing_config);
    quadController.set_plan(*plan);
    
    std::unique_ptr<Simulator> s(new Simulator({time_step_us, 1, true}));
    StandaloneDrone d{fixed_wing_config, s->simulation_clock, quadController};

    d.set_fake_ground_level(5);
    d.set_drone_state_processor(*s);
    s->add_environment_object(d);
    s->add_drone_state_processor(&dsLog);
    s->start(quadController.get_total_plan_duration_us());
    
    return 0;
}

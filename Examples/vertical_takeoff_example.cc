
#include "../src/StandaloneDrone.h"
#include "../src/Simulator.h"
#include "../src/Controllers/SimpleFixedWingController.h"
#include "../src/Logging/DroneStateLogger.h"
#include <Eigen/Eigen>


int main()
{
    double time_step_s = 0.004;
    double epoch_s = 10;
    const char* fixed_wing_config = "../drone_models/fixed_wing";

    Manoeuvre manoeuvres[2] = {Manoeuvre::HOLD, Manoeuvre::CLIMB};
    boost::chrono::microseconds section_lenghts[2] = {
        boost::chrono::microseconds{1000000 * 3}, // 3 s
        boost::chrono::microseconds{1000000 * 2}, // 2 s
    };
    ManoeuvrePlan plan{(boost::chrono::microseconds*)&section_lenghts, (Manoeuvre*)&manoeuvres, 2};
    SimpleFixedWingController quadController{config_from_file_path(fixed_wing_config)};
    quadController.set_plan(plan);

    DroneStateLogger dsLog;

    std::unique_ptr<Simulator> s(new Simulator({static_cast<long>(time_step_s * 1000000), 1, true}));
    StandaloneDrone d{fixed_wing_config, s->simulation_clock, quadController};
    d.set_fake_ground_level(5);
    d.set_drone_state_processor(*s);
    s->add_environment_object(d);
    s->add_drone_state_processor(&dsLog);
    s->start(quadController.get_total_plan_duration_us());
    
    return 0;
}


#include "../src/StandaloneDrone.h"
#include "../src/Simulator.h"
#include "../src/IrisQuadController.h"
#include <Eigen/Eigen>

double sigmoid(double x) {
    return 1.0 / (1.0 + pow(M_E, -x));
}

double pmw_signal(double dt) {
    return sigmoid(dt) - sigmoid(0); 
}


int main()
{
    double time_step_s = 0.004;
    double epoch_s = 10;
    const char* fixed_wing_config = "../drone_models/fixed_wing";

    Manoeuvre manoeuvres[2] = {Manoeuvre::CLIMB, Manoeuvre::HOLD};
    boost::chrono::microseconds section_lenghts[2] = {
        boost::chrono::microseconds{1000000 * 3}, // 3 s
        boost::chrono::microseconds{1000000 * 2}, // 2 s
    };
    ManoeuvrePlan plan{(boost::chrono::microseconds*)&section_lenghts, (Manoeuvre*)&manoeuvres, 2};
    IrisQuadController quadController{config_from_file_path(fixed_wing_config)};
    quadController.set_plan(plan);

    std::unique_ptr<Simulator> s(new Simulator({static_cast<long>(time_step_s * 1000000), 1, true}));
    StandaloneDrone d{fixed_wing_config, s->simulation_clock, quadController};
    s->add_environment_object(d);
    s->start(quadController.get_total_plan_duration_us());
    
    return 0;
}

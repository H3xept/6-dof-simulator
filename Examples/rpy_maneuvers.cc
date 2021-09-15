
#include "../src/StandaloneDrone.h"
#include "../src/Simulator.h"



int main()
{
    const char* fixed_wing_config = "../drone_models/fixed_wing";

    std::unique_ptr<Simulator> s(new Simulator({4000, 1, true}));
    StandaloneDrone d{fixed_wing_config, s->simulation_clock};
    s->add_environment_object(d);
    s->start();
    
    return 0;
}

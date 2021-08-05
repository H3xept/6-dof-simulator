#include "Sim6DOFInfo.h"
#include "Logging/ConsoleLogger.h"
#include "FlatEarth.h"
#include "Simulator.h"
#include "Drone.h"

int main(int argc, char const *argv[])
{
    ConsoleLogger cl{};
    cl.set_debug(true);
    cl.log("Normal log");
    cl.err_log("Error log");

    FlatEarth f{cl};
    printf("%s\n", f.str().c_str());

    Simulator s{{100, 1}};
    printf("%s\n", s.str().c_str());
    s.start();
    s.pause();
    s.resume();

    Drone d{"../drone_models/fixed_wing"};
    printf("%s\n", d.get_config().str().c_str());
    return 0;
}

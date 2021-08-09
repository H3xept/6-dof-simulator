#include "Sim6DOFInfo.h"
#include "Logging/ConsoleLogger.h"
#include "Simulator.h"
#include "Drone.h"
#include <boost/thread.hpp>

#include "Sockets/MAVLinkConnectionHandler.h"

int main(int argc, char const *argv[])
{
    ConsoleLogger cl{};
    cl.set_debug(true);
    cl.log("Normal log");
    cl.err_log("Error log");

    boost::asio::io_service service;
    MAVLinkConnectionHandler handler{service, ConnectionTarget::PX4};
    boost::thread link_thread = boost::thread(boost::bind(&boost::asio::io_service::run, &service));
    Simulator s{{100, 1}};
    Drone d{"../drone_models/fixed_wing", handler};
    s.add_environment_object(d);
    s.start();

    // s.pause();
    // s.resume();
    // service.run();
    
    return 0;
}

#include "Sim6DOFInfo.h"
#include "Logging/ConsoleLogger.h"
#include "Simulator.h"
#include "Drone.h"
#include "StandaloneDrone.h"
#include <boost/thread.hpp>
#include "Sockets/MAVLinkConnectionHandler.h"
#include <Eigen/Eigen>

int main()
{

    std::cout << "6 DOF Simulator" << std::endl;

    const char* fixed_wing_config = "../drone_models/small";
    ConsoleLogger* cl = ConsoleLogger::shared_instance();
    cl->set_debug(false);

    boost::asio::io_service service;
    MAVLinkConnectionHandler handler{service, ConnectionTarget::PX4};
    boost::thread link_thread = boost::thread(boost::bind(&boost::asio::io_service::run, &service));
    std::unique_ptr<Simulator> s(new Simulator({4000, 1, true}));
    
    Drone d{fixed_wing_config, handler, s->simulation_clock };
    d.set_fake_ground_level(0);
    s->add_environment_object(d);
    s->start();
    
    return 0;
}

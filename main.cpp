#include <icubAirHockeyControlModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <iomanip>


int main(int argc, char *argv[])
{
    yarp::os::Network net;
    if (!net.checkNetwork()){
        throw std::runtime_error("Could not init yarp net");
    }

    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);
    
    IcubAirHockeyControlModule module;
    return module.runModule(rf);

    return 0;
}

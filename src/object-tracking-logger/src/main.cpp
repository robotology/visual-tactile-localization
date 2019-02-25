#include <Logger.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include <cstdlib>

using namespace yarp::os;


int main(int argc, char** argv)
{
    const std::string log_ID = "[Main]";
    yInfo() << log_ID << "Configuring and starting module...";

    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError() << log_ID << "Yarp is not available.";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.setVerbose();
    rf.setDefaultContext("object-tracking-logger");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc, argv);

    /* Get period. */
    const double period = rf.check("period", Value(0.05)).asDouble();

    /* Run module. */
    ObjectTrackingLogger logger("object-tracking-logger", period);

    logger.runModule(rf);

    return EXIT_SUCCESS;
}

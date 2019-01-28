#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>

#include <SFM.h>

#include <string>

using namespace yarp::os;

class Depth : public yarp::os::RFModule
{
public:
    Depth(const std::string port_prefix) :
        port_prefix_(port_prefix),
        sfm_(port_prefix_)
    { }

    bool configure(ResourceFinder& rf)
    {
        period_ = rf.check("period", Value(1.0)).asDouble();

        yInfo() << log_ID_ << "Period is:" << period_;

        ResourceFinder rf_sfm;
        rf_sfm.setVerbose(true);
        rf_sfm.setDefaultConfigFile("sfm_config.ini");
        rf_sfm.setDefaultContext("object-tracking");
        rf_sfm.configure(0, NULL);

        return sfm_.configure(rf_sfm);
    }

    double getPeriod()
    {
        return period_;
    }

    bool updateModule()
    {
        sfm_.updateDepth();

        return true;
    }

    bool close()
    {
        return sfm_.close();
    }

private:
    const std::string log_ID_ = "[DEPTH]";

    const std::string port_prefix_;

    double period_;

    SFM sfm_;
};

int main(int argc, char** argv)
{
    const std::string log_ID = "[MAIN]";
    const std::string port_prefix = "object-tracking-depth";

    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError() << log_ID << "Yarp is not available.";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("config.ini");
    rf.setDefaultContext("object-tracking-depth");
    rf.configure(argc, argv);

    Depth depth(port_prefix);
    depth.runModule(rf);

    return EXIT_SUCCESS;
}



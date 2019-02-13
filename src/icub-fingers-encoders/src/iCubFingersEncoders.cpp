#include <iCubFingersEncoders.h>

#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>

#include <iostream>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;


iCubFingersEncoders::iCubFingersEncoders
(
    const std::string context,
    const std::string laterality,
    const std::string port_prefix
) :
    use_interface_(false)
{
    // Load configuration from config file
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("analogs_configuration.ini");
    rf.setDefaultContext(context);
    rf.configure(0, NULL);

    // Get inner resource finder according to requested laterality
    if ((laterality != "right") && (laterality != "left"))
    {
        throw std::runtime_error("ERROR::" + log_ID_ + "::CTOR::\nERROR: invalid laterality" + laterality + ".");
    }
    ResourceFinder inner_rf = rf.findNestedResourceFinder(laterality.c_str());
    bool use_bounds_ = inner_rf.check("use_bounds", Value(false)).asBool();
    if (use_bounds_)
    {
        bool valid_vector;
        Vector bounds_col_0;
        std::tie(valid_vector, bounds_col_0) = loadVectorDouble(inner_rf, "bounds_col_0", 16);
        if (!valid_vector)
        {
            throw std::runtime_error("ERROR::" + log_ID_ + "::CTOR::\nERROR: bounds requested but not availalbe in the configuration file.");
        }

        Vector bounds_col_1;
        std::tie(valid_vector, bounds_col_1) = loadVectorDouble(inner_rf, "bounds_col_1", 16);
        if (!valid_vector)
        {
            throw std::runtime_error("ERROR::" + log_ID_ + "::CTOR::\nERROR: bounds requested but not availalbe in the configuration file.");
        }

        analog_bounds_.resize(16, 2);
        analog_bounds_.setCol(0, bounds_col_0);
        analog_bounds_.setCol(1, bounds_col_1);
    }

    // Try to use the PolyDriver interface
    Property prop_analog;
    prop_analog.put("device", "analogsensorclient");
    prop_analog.put("local", "/" + port_prefix + "/" + laterality + "_hand/analog:i");
    prop_analog.put("remote", "/icub/" + laterality + "_hand/analog:o");
    if (drv_analog_.open(prop_analog))
    {
        // Try to retrieve the view
        if (drv_analog_.view(ianalog_) && ianalog_ != nullptr)
            use_interface_ = true;
    }

    if (!use_interface_)
    {
        std::cout << "INFO::" + log_ID_ + "::CTOR::\nINFO: PolyDriver interface IAnalogSensors not available. Using raw encoders from a port." << std::endl;

        // If the PolyDriver is not available, use a standard port
        if (!port_analogs_.open("/" + port_prefix + "/" + laterality + "_hand/analog:i"))
        {
            throw std::runtime_error("ERROR::" + log_ID_ + "::CTOR::\nERROR: unable to open port for analog encoders.");
        }
    }
}


iCubFingersEncoders::~iCubFingersEncoders()
{
    if (!use_interface_)
        port_analogs_.close();
}


std::pair<bool, Vector> iCubFingersEncoders::getEncoders()
{
    Vector analogs(16);
    bool outcome = false;

    if (use_interface_)
    {
        outcome = (ianalog_->read(analogs)) == (IAnalogSensor::AS_OK);
    }
    else
    {
        Bottle* bottle_analogs =  port_analogs_.read(true);

        if (bottle_analogs != nullptr)
        {
            for (size_t i = 0; i < 16; ++i)
                analogs(i) = bottle_analogs->get(i).asDouble();

            outcome = true;
        }
    }

    return std::make_pair(outcome, analogs);
}


std::pair<bool, Matrix> iCubFingersEncoders::getAnalogBounds()
{
    if (!use_bounds_)
        return std::make_pair(false, Matrix());

    return std::make_pair(true, analog_bounds_);
}


std::pair<bool, yarp::sig::Vector> iCubFingersEncoders::loadVectorDouble
(
    const ResourceFinder& rf,
    const std::string key,
    const std::size_t size
)
{
    bool ok = true;

    if (rf.find(key).isNull())
        ok = false;

    Bottle* b = rf.find(key).asList();
    if (b == nullptr)
        ok = false;

    if (b->size() != size)
        ok = false;

    if (!ok)
        return std::make_pair(false, Vector());

    Vector vector(size);
    for (std::size_t i = 0; i < b->size(); i++)
    {
        Value item_v = b->get(i);
        if (item_v.isNull())
            return std::make_pair(false, Vector());

        if (!item_v.isDouble())
            return std::make_pair(false, Vector());

        vector(i) = item_v.asDouble();
    }

    return std::make_pair(true, vector);
}

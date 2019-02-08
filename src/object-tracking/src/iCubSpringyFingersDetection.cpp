#include <iCubSpringyFingersDetection.h>

#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>

using namespace yarp::os;
using namespace yarp::sig;


iCubSpringyFingersDetection::iCubSpringyFingersDetection(const std::string laterality)
{
    // Form the name of the configuration file
    const std::string config_filename = "springy_calibration_" + laterality + ".ini";

    // Get path of the calibration file
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("object-tracking");
    rf.setDefaultConfigFile(config_filename.c_str());
    rf.configure(0, NULL);

    // Initialize springy fingers detection
    Property prop;
    const std::string calibration_file_path = rf.findFile(config_filename);
    prop.fromConfigFile(calibration_file_path.c_str());
    springy_model_.fromProperty(prop);

    if (!springy_model_.isCalibrated())
    {
        std::string err = "ICUBSPRINGYFINGERSDETECTION::CTOR::ERROR\n\tError: cannot properly initialize calibration of springy fingers modelling.";
        throw(std::runtime_error(err));
    }

    // Load thresholds for contact detection
    bool valid_threshold;
    std::tie(valid_threshold, thresholds_) = loadVectorDouble(rf.findNestedResourceFinder("thresholds"), "thresholds", 5);
    if (!valid_threshold)
    {
        std::string err = "ICUBSPRINGYFINGERSDETECTION::CTOR::ERROR\n\tError: cannot load threshold for contact detection with springy fingers modelling.";
        throw(std::runtime_error(err));
    }
}


iCubSpringyFingersDetection::~iCubSpringyFingersDetection()
{ }


std::pair<bool, std::unordered_map<std::string, bool>> iCubSpringyFingersDetection::getActiveFingers()
{
    // Get the output of the springy LSSVM model
    Value springy_output;
    Bottle* springy_list;
    springy_model_.getOutput(springy_output);
    springy_list = springy_output.asList();

    std::unordered_map<std::string, bool> contacts;
    // Reset contacts
    for (std::size_t i = 0; i < 5; i++)
        contacts[fingers_names_.at(i)] = false;

    // Check if there is contact
    for (std::size_t i = 0; i < 5; i++)
    {
        if (springy_list->get(i).asDouble() > thresholds_(i))
            contacts.at(fingers_names_.at(i)) = true;
    }

    return std::make_pair(true, contacts);
}


std::pair<bool, yarp::sig::Vector> iCubSpringyFingersDetection::loadVectorDouble
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

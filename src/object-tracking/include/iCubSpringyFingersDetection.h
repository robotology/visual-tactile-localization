#ifndef ICUBSPRINGYFINGERSDETECTION_H
#define ICUBSPRINGYFINGERSDETECTION_H

#include <ContactDetection.h>

#include <yarp/sig/Vector.h>

#include <iCub/perception/springyFingers.h>


class iCubSpringyFingersDetection : public ContactDetection
{
public:
    iCubSpringyFingersDetection(const std::string laterality);

    virtual ~iCubSpringyFingersDetection();

    std::pair<bool, std::unordered_map<std::string, bool>> getActiveFingers() override;

protected:
    std::pair<bool, yarp::sig::Vector> loadVectorDouble(const yarp::os::ResourceFinder& rf, const std::string key, const std::size_t size);

    iCub::perception::SpringyFingersModel springy_model_;

    yarp::sig::Vector thresholds_;
};

#endif /* ICUBSPRINGYFINGERSDETECTION_H */

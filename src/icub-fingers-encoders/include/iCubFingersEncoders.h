#ifndef ICUBFINGERSENCODERS_H
#define ICUBFINGERSENCODERS_H

#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/ResourceFinder.h>

#include <string>

class iCubFingersEncoders
{
public:
    iCubFingersEncoders(const std::string context, const std::string laterality, const std::string port_prefix);

    virtual ~iCubFingersEncoders();

    std::pair<bool, yarp::sig::Vector> getEncoders();

    std::pair<bool, yarp::sig::Matrix> getAnalogBounds();

protected:
    std::pair<bool, yarp::sig::Vector> loadVectorDouble(const yarp::os::ResourceFinder& rf, const std::string key, const std::size_t size);

    /**
     * Indicates whether the PolyDriver interface is available.
     * If not available, the encoder port is used instead.
     */
    bool use_interface_;

    /**
     * To be used if the driver is available, e.g. on the real robot.
     */
    yarp::dev::PolyDriver drv_analog_;

    yarp::dev::IAnalogSensor *ianalog_;

    /**
     * To be used if the driver is not available, e.g. using dumped data.
     */
    yarp::os::BufferedPort<yarp::os::Bottle> port_analogs_;

    /**
     * Optional analog bounds.
     */
    yarp::sig::Matrix analog_bounds_;
    bool use_bounds_;

    const std::string log_ID_ = "ICUBFINGERSENCODERS";
};

#endif /* ICUBFINGERSENCODERS_H */

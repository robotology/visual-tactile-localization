#ifndef GAZE_CONTROLLER_H
#define GAZE_CONTROLLER_H

#include <yarp/dev/GazeControl.h>
#include <yarp/dev/PolyDriver.h>

class GazeController
{
public:
    GazeController(const std::string &port_prefix);

    virtual ~GazeController();

    bool getCameraPose(const std::string eye_name, yarp::sig::Vector &pos, yarp::sig::Vector &att);

    bool getCameraIntrinsics(const std::string eye_name, double &fx, double &fy, double &cx, double &cy);

private:
    yarp::dev::PolyDriver drv_gaze;

    yarp::dev::IGazeControl *igaze;
};

#endif /* GAZE_CONTROLLER_H */

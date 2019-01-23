#ifndef GAZE_CONTROLLER_H
#define GAZE_CONTROLLER_H

#include <iCub/iKin/iKinFwd.h>

#include <yarp/dev/GazeControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>


class GazeController
{
public:
    GazeController(const std::string port_prefix);

    virtual ~GazeController();

    bool getEyesConfiguration(yarp::sig::Vector& eye_enc);

    bool getCameraPoses(yarp::sig::Vector& pos_left, yarp::sig::Vector& att_left, yarp::sig::Vector& pos_right, yarp::sig::Vector& att_right);

    bool getCameraIntrinsics(const std::string eye_name, double &fx, double &fy, double &cx, double &cy);

    bool isGazeInterfaceAvailable();

    yarp::dev::IGazeControl& getGazeInterface();

private:
    yarp::dev::PolyDriver drv_gaze;

    yarp::dev::IGazeControl *igaze;

    bool use_igaze;

    bool use_ienc;

    yarp::os::BufferedPort<yarp::os::Bottle> port_head_enc_;

    yarp::os::BufferedPort<yarp::os::Bottle> port_torso_enc_;

    iCub::iKin::iCubEye icub_kin_eye_left_;

    iCub::iKin::iCubEye icub_kin_eye_right_;

    double fx_left_;

    double fy_left_;

    double cx_left_;

    double cy_left_;

    double fx_right_;

    double fy_right_;

    double cx_right_;

    double cy_right_;
};

#endif /* GAZE_CONTROLLER_H */

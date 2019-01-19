#include <GazeController.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Bottle.h>
#include <yarp/math/Math.h>

using namespace yarp::os;


GazeController::GazeController(const std::string &port_prefix)
{
    /**
     * Drivers configuration
     */

    // prepare properties for the GazeController
    Property prop;
    prop.put("device", "gazecontrollerclient");
    prop.put("remote", "/iKinGazeCtrl");
    prop.put("local", port_prefix + "/gazecontroller");

    // let's give the controller some time to warm up
    bool driver_ok = false;
    double t0 = SystemClock::nowSystem();
    while (SystemClock::nowSystem() - t0 < 10.0)
    {
        // this might fail if controller
        // is not connected to solver yet
        if (drv_gaze.open(prop))
        {
            driver_ok= true;
            break;
        }
        SystemClock::delaySystem(1.0);
    }
    if (!driver_ok)
    {
        std::string err = "GAZECONTROLLER::CTOR::ERROR\n\tError: cannot open the Gaze controller driver.";
        throw(std::runtime_error(err));
    }

    // try to retrieve the view
    driver_ok = drv_gaze.view(igaze);
    if (!driver_ok || igaze == nullptr)
    {
        std::string err = "GAZECONTROLLER::CTOR::ERROR\n\tError: cannot retrieve the Gaze controller driver.";
        throw(std::runtime_error(err));
    }
}


GazeController::~GazeController()
{
    // close the driver
    drv_gaze.close();
}


bool GazeController::getCameraPose
(
    const std::string eye_name,
    yarp::sig::Vector &pos,
    yarp::sig::Vector &att
)
{
    if ((eye_name != "right") && (eye_name != "left"))
        return false;

    bool valid_pose = false;

    if (eye_name == "right")
        valid_pose = igaze->getRightEyePose(pos, att);
    else if(eye_name == "left")
        valid_pose = igaze->getLeftEyePose(pos, att);

    return valid_pose;
}


bool GazeController::getCameraIntrinsics
(
    const std::string eye_name,
    double &fx,
    double &fy,
    double &cx,
    double &cy
)
{
    Bottle info;
    igaze->getInfo(info);
    std::string key = "camera_intrinsics_" + eye_name;

    if (info.find(key).isNull())
        return false;

    Bottle *list = info.find("camera_intrinsics_" + eye_name).asList();

    fx = list->get(0).asDouble();
    cx = list->get(2).asDouble();
    fy = list->get(5).asDouble();
    cy = list->get(6).asDouble();

    return true;
}

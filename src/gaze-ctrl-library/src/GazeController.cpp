#include <GazeController.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/math/Math.h>

#include <cmath>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::iKin;

GazeController::GazeController(const std::string port_prefix)
{
    use_ienc = false;

    /**
     * Drivers configuration
     */

    // Prepare properties for the GazeController
    Property prop;
    prop.put("device", "gazecontrollerclient");
    prop.put("remote", "/iKinGazeCtrl");
    prop.put("local", "/" + port_prefix + "/gazecontroller");

	// let's give the controller some time to warm up
    bool ok = false;
    int number_temptatives = 5;
    while (number_temptatives > 0)
    {
        // this might fail if controller
        // is not connected to solver yet
        if (drv_gaze.open(prop))
        {
            ok = true;
            break;
        }
        yarp::os::SystemClock::delaySystem(1.0);
		number_temptatives--;
    }

    // try to retrieve the view
    if (ok)
    {
        ok &= drv_gaze.view(igaze);
	    ok &= (igaze != nullptr);
    }

    if (!ok)
    {
        yWarning() << "GAZECONTROLLER::CTOR. Warning: cannot open the Gaze controller driver, switching to the encoders.";

        use_igaze = false;

        // Open ports
        port_head_enc_.open("/" + port_prefix + "/icub/head:i");
        port_torso_enc_.open("/" + port_prefix + "/icub/torso:i");

        // Initialize forward kinematics
        icub_kin_eye_left_ = iCubEye("left_v2");
        icub_kin_eye_left_.setAllConstraints(false);
        icub_kin_eye_left_.releaseLink(0);
        icub_kin_eye_left_.releaseLink(1);
        icub_kin_eye_left_.releaseLink(2);

        icub_kin_eye_right_ = iCubEye("right_v2");
        icub_kin_eye_right_.setAllConstraints(false);
        icub_kin_eye_right_.releaseLink(0);
        icub_kin_eye_right_.releaseLink(1);
        icub_kin_eye_right_.releaseLink(2);

        // Also load instrinsic parameters from configuration file
        // installed by the package 'object-tracking'
        ResourceFinder rf;
        rf.setVerbose(true);
        rf.setDefaultContext("object-tracking");
        rf.setDefaultConfigFile("sfm_config.ini");
        rf.configure(0, nullptr);

        ResourceFinder rf_intrinsics_left = rf.findNestedResourceFinder("CAMERA_CALIBRATION_LEFT");
        fx_left_ = rf_intrinsics_left.check("fx", Value(234.666)).asDouble();
        fy_left_ = rf_intrinsics_left.check("fy", Value(234.25)).asDouble();
        cx_left_ = rf_intrinsics_left.check("cx", Value(149.795)).asDouble();
        cy_left_ = rf_intrinsics_left.check("cy", Value(123.059)).asDouble();

		yInfo() << "GAZECONTROLLER::CTOR. fx_left_ from configuration file is:" << fx_left_;
		yInfo() << "GAZECONTROLLER::CTOR. fy_left_ from configuration file is:" << fy_left_;
		yInfo() << "GAZECONTROLLER::CTOR. cx_left_ from configuration file is:" << cx_left_;
		yInfo() << "GAZECONTROLLER::CTOR. cy_left_ from configuration file is:" << cy_left_;

        ResourceFinder rf_intrinsics_right = rf.findNestedResourceFinder("CAMERA_CALIBRATION_RIGHT");
        fx_right_ = rf_intrinsics_right.check("fx", Value(234.88)).asDouble();
        fy_right_ = rf_intrinsics_right.check("fy", Value(234.582)).asDouble();
        cx_right_ = rf_intrinsics_right.check("cx", Value(160.77)).asDouble();
        cy_right_ = rf_intrinsics_right.check("cy", Value(123.491)).asDouble();

		yInfo() << "GAZECONTROLLER::CTOR. fx_right_ from configuration file is:" << fx_right_;
		yInfo() << "GAZECONTROLLER::CTOR. fy_right_ from configuration file is:" << fy_right_;
		yInfo() << "GAZECONTROLLER::CTOR. cx_right_ from configuration file is:" << cx_right_;
		yInfo() << "GAZECONTROLLER::CTOR. cy_right_ from configuration file is:" << cy_right_;
    }
    else
    {
        use_igaze = true;
    }
}


GazeController::~GazeController()
{
    if (use_igaze)
    {
        // Close the driver
        drv_gaze.close();
    }
    else
    {
        // Close the ports
        port_head_enc_.close();
        port_torso_enc_.close();
    }
}


bool GazeController::getEyesConfiguration(Vector& eye_enc)
{
    if (use_ienc)
    {
        // TODO
        return false;
    }
    else
    {
        eye_enc.resize(3);

        Bottle* bottle_head  = port_head_enc_.read(true);
        if (bottle_head == nullptr)
            return false;

        for (std::size_t i = 0; i < 3; i++)
            eye_enc(i) = bottle_head->get(i + 3).asDouble();

        return true;
    }
}


bool GazeController::getCameraPoses
(
    yarp::sig::Vector& pos_left,
    yarp::sig::Vector& att_left,
    yarp::sig::Vector& pos_right,
    yarp::sig::Vector& att_right
)
{
    if (use_igaze)
    {
        return igaze->getLeftEyePose(pos_left, att_left) &&
               igaze->getRightEyePose(pos_right, att_right);
    }
    else
    {
        Vector root_eye_enc(8, 0.0);

        Bottle* bottle_torso = port_torso_enc_.read(true);
        Bottle* bottle_head  = port_head_enc_.read(true);
        if (!bottle_torso || !bottle_head)
            return false;

        // Torso in reversed order
        root_eye_enc(0) = bottle_torso->get(2).asDouble();
        root_eye_enc(1) = bottle_torso->get(1).asDouble();
        root_eye_enc(2) = bottle_torso->get(0).asDouble();

        // Neck and eyes tilt
        for (size_t i = 0; i < 4; ++i)
            root_eye_enc(3 + i) = bottle_head->get(i).asDouble();

        // Left eye dof from version and vergence
        root_eye_enc(7) = bottle_head->get(4).asDouble() + bottle_head->get(5).asDouble() / 2.0;

        // Extract pose
        Vector left_eye_pose = icub_kin_eye_left_.EndEffPose(M_PI / 180.0 * root_eye_enc);

        pos_left.resize(3);
        pos_left[0] = left_eye_pose[0];
        pos_left[1] = left_eye_pose[1];
        pos_left[2] = left_eye_pose[2];

        att_left.resize(4);
        att_left[0] = left_eye_pose[3];
        att_left[1] = left_eye_pose[4];
        att_left[2] = left_eye_pose[5];
        att_left[3] = left_eye_pose[6];

        // Right eye dof from version and vergence
        root_eye_enc(7) = bottle_head->get(4).asDouble() - bottle_head->get(5).asDouble() / 2.0;
        Vector right_eye_pose = icub_kin_eye_right_.EndEffPose(M_PI / 180.0 * root_eye_enc);

        // Extract pose
        pos_right.resize(3);
        pos_right[0] = right_eye_pose[0];
        pos_right[1] = right_eye_pose[1];
        pos_right[2] = right_eye_pose[2];

        att_right.resize(4);
        att_right[0] = right_eye_pose[3];
        att_right[1] = right_eye_pose[4];
        att_right[2] = right_eye_pose[5];
        att_right[3] = right_eye_pose[6];

        return true;
    }
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
    if ((eye_name != "left") && (eye_name != "right"))
        return false;

    if (use_igaze)
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
    else
    {
        if (eye_name == "left")
        {
            fx = fx_left_;
            fy = fy_left_;
            cx = cx_left_;
            cy = cy_left_;
        }
        else if (eye_name == "right")
        {
            fx = fx_right_;
            fy = fy_right_;
            cx = cx_right_;
            cy = cy_right_;
        }

        return true;
    }
}


bool GazeController::isGazeInterfaceAvailable()
{
    return use_igaze;
}


IGazeControl& GazeController::getGazeInterface()
{
    return *igaze;
}

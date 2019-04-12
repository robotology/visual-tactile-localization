/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <iCubCamera.h>

#include <yarp/cv/Cv.h>
#include <yarp/eigen/Eigen.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Image.h>

using namespace Eigen;
using namespace iCub::iKin;
using namespace yarp::cv;
using namespace yarp::eigen;
using namespace yarp::os;
using namespace yarp::sig;


iCubCamera::iCubCamera
(
    const std::string& laterality,
    const std::string& port_prefix,
    const std::string& context
) :
    laterality_(laterality)
{
    if ((laterality_ != "left") && (laterality_ != "right"))
    {
        std::string err = log_ID_ + "::ctor. Please use a valid laterality when constructing the iCubCamera instance.";
        throw(std::runtime_error(err));
    }

    // Prepare properties for the GazeController
    Property prop_gaze;
    prop_gaze.put("device", "gazecontrollerclient");
    prop_gaze.put("remote", "/iKinGazeCtrl");
    prop_gaze.put("local", "/" + port_prefix + "/gazecontroller");

    // Let's give the controller some time to warm up
    bool ok = false;
    int number_temptatives = 3;
    while (number_temptatives > 0)
    {
        // This might fail if controller is not connected to solver yet
        if (drv_gaze_.open(prop_gaze))
        {
            ok = true;
            break;
        }
        yarp::os::SystemClock::delaySystem(1.0);
        number_temptatives--;
    }

    // Try to retrieve the required view
    if (ok)
    {
        ok &= drv_gaze_.view(igaze_);
        ok &= (igaze_ != nullptr);

        Bottle info;
        igaze_->getInfo(info);

        // Load camera resolution
        std::string key = "camera_width_" + laterality_;
        if (info.find(key).isNull())
        {
            std::string err = log_ID_ + "::ctor. Error: cannot load iCub " + laterality_ + " camera width.";
            throw(std::runtime_error(err));
        }
        parameters_.width = info.find(key).asInt();


        key = "camera_height_" + laterality_;
        if (info.find(key).isNull())
        {
            std::string err = log_ID_ + "::ctor. Error: cannot load iCub " + laterality_ + " camera height.";
            throw(std::runtime_error(err));
        }
        parameters_.height = info.find(key).asInt();


        // Load intrinsic parameters
        key = "camera_intrinsics_" + laterality_;

        if (info.find(key).isNull())
        {
            std::string err = log_ID_ + "::ctor. Error: cannot load iCub " + laterality_ + " camera intrinsic parameters.";
            throw(std::runtime_error(err));
        }

        Bottle *list = info.find(key).asList();

        parameters_.fx = list->get(0).asDouble();
        parameters_.cx = list->get(2).asDouble();
        parameters_.fy = list->get(5).asDouble();
        parameters_.cy = list->get(6).asDouble();

        parameters_.initialized = true;
    }

    if (ok)
        yInfo() << log_ID_ << "::ctor. Using the Gaze interface.";
    else
    {
        yWarning() << log_ID_ << "::ctor. Warning: cannot open the Gaze controller driver, switching to raw encoders.";

        use_igaze_ = false;

        // Load camera resolution and instrinsic parameters from configuration file
        ResourceFinder rf;
        rf.setVerbose(true);
        rf.setDefaultContext(context);
        rf.setDefaultConfigFile("icub_camera_config.ini");
        rf.configure(0, nullptr);

        ResourceFinder rf_laterality = rf.findNestedResourceFinder(laterality_.c_str());
        ok =  rf_laterality.check("width");
        ok &= rf_laterality.check("height");
        ok &= rf_laterality.check("fx");
        ok &= rf_laterality.check("fy");
        ok &= rf_laterality.check("cx");
        ok &= rf_laterality.check("cy");
        if (!ok)
        {
            std::string err = log_ID_ + "::ctor. Error: cannot load iCub " + laterality_ + " camera parameters.";
            throw(std::runtime_error(err));
        }

        parameters_.width = rf_laterality.find("width").asDouble();
        parameters_.height = rf_laterality.find("height").asDouble();
        parameters_.fx = rf_laterality.find("fx").asDouble();
        parameters_.fy = rf_laterality.find("fy").asDouble();
        parameters_.cx = rf_laterality.find("cx").asDouble();
        parameters_.cy = rf_laterality.find("cy").asDouble();
        parameters_.initialized = true;

        // Open ports
        if (!port_head_enc_.open("/" + port_prefix + "/icub/head:i"))
        {
            std::string err = log_ID_ + "::ctor. Error: cannot open iCub head encoders.";
            throw(std::runtime_error(err));
        }
        if (!port_torso_enc_.open("/" + port_prefix + "/icub/torso:i"))
        {
            std::string err = log_ID_ + "::ctor. Error: cannot open iCub torso encoders.";
            throw(std::runtime_error(err));
        }

        // Initialize forward kinematics
        icub_eye_fk_ = iCubEye(laterality_ + "_v2");
        icub_eye_fk_.setAllConstraints(false);
        icub_eye_fk_.releaseLink(0);
        icub_eye_fk_.releaseLink(1);
        icub_eye_fk_.releaseLink(2);
    }

    // Log parameters
    std::cout << log_ID_ + "::ctor. Camera " + laterality_ + " parameters." << std::endl;
    std::cout << log_ID_ + "    - width: " << parameters_.width << std::endl;
    std::cout << log_ID_ + "    - height: " << parameters_.height << std::endl;
    std::cout << log_ID_ + "    - fx: " << parameters_.fx << std::endl;
    std::cout << log_ID_ + "    - fy: " << parameters_.fy << std::endl;
    std::cout << log_ID_ + "    - cx: " << parameters_.cx << std::endl;
    std::cout << log_ID_ + "    - cy: " << parameters_.cy << std::endl;

    // Open rgb input port
    if (!(port_rgb_in_.open("/" + port_prefix + "/rgbImage:i")))
    {
        std::string err = log_ID_ + "::ctor. Error: cannot open iCub " + laterality_ + "rgb camera input port.";
        throw(std::runtime_error(err));
    }

    // Open depth input port
    if (laterality_ == "right")
    {
        yWarning() << log_ID_ + "::ctor. Warning: depth camera not available for iCub right eye";
    }
    else if (!(port_depth_in_.open("/" + port_prefix + "/depthImage:i")))
    {
        depth_available = false;

        std::string err = log_ID_ + "::ctor. Error: cannot open iCub " + laterality_ + "depth camera input port.";
        throw(std::runtime_error(err));
    }
}


iCubCamera::~iCubCamera()
{
    if (use_igaze_)
    {
        drv_gaze_.close();
    }
    else
    {
        port_head_enc_.close();
        port_torso_enc_.close();
    }

    port_rgb_in_.close();

    if (depth_available)
        port_depth_in_.close();
}


std::pair<bool, Transform<double, 3, Affine>> iCubCamera::getCameraPose(const bool& blocking)
{
    Transform<double, 3, Affine> pose;

    yarp::sig::Vector pos_yarp;
    yarp::sig::Vector att_yarp;

    bool ok = false;

    if (use_igaze_)
    {
        if (laterality_ == "left")
            ok = igaze_->getLeftEyePose(pos_yarp, att_yarp);
        else
            ok = igaze_->getRightEyePose(pos_yarp, att_yarp);
    }
    else
    {
        yarp::sig::Vector root_eye_enc(8, 0.0);

        Bottle* bottle_torso = port_torso_enc_.read(blocking);
        Bottle* bottle_head  = port_head_enc_.read(blocking);
        if (bottle_torso && bottle_head)
        {
            ok = true;

            // Torso in reversed order
            root_eye_enc(0) = bottle_torso->get(2).asDouble();
            root_eye_enc(1) = bottle_torso->get(1).asDouble();
            root_eye_enc(2) = bottle_torso->get(0).asDouble();

            // Neck and eyes tilt
            for (size_t i = 0; i < 4; ++i)
                root_eye_enc(3 + i) = bottle_head->get(i).asDouble();

            // Left eye dof from version and vergence
            if (laterality_ == "left")
                root_eye_enc(7) = bottle_head->get(4).asDouble() + bottle_head->get(5).asDouble() / 2.0;
            else
                root_eye_enc(7) = bottle_head->get(4).asDouble() - bottle_head->get(5).asDouble() / 2.0;

            // Extract position
            yarp::sig::Vector pose = icub_eye_fk_.EndEffPose(M_PI / 180.0 * root_eye_enc);

            pos_yarp.resize(3);
            pos_yarp[0] = pose[0];
            pos_yarp[1] = pose[1];
            pos_yarp[2] = pose[2];

            att_yarp.resize(4);
            att_yarp[0] = pose[3];
            att_yarp[1] = pose[4];
            att_yarp[2] = pose[5];
            att_yarp[3] = pose[6];
        }
    }

    if (!ok)
        return std::make_pair(false, Transform<double, 3, Affine>());

    pose = Translation<double, 3>(toEigen(pos_yarp));
    pose.rotate(AngleAxisd(att_yarp(3), toEigen(att_yarp).head<3>()));

    return std::make_pair(true, pose);
}


std::pair<bool, cv::Mat> iCubCamera::getRgbImage(const bool& blocking)
{
    // Get image
    ImageOf<PixelRgb>* image_in;
    image_in = port_rgb_in_.read(blocking);

    if (image_in == nullptr)
        return std::make_pair(false, cv::Mat());

    cv::Mat image = yarp::cv::toCvMat(*image_in);
    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

    return std::make_pair(true, image);
}


std::pair<bool, MatrixXf> iCubCamera::getDepthImage(const bool& blocking)
{
    // Get image
    ImageOf<PixelFloat>* image_in;
    image_in = port_depth_in_.read(blocking);

    if (image_in == nullptr)
        return std::make_pair(false, MatrixXf());

    cv::Mat image = yarp::cv::toCvMat(*image_in);
    Map<Eigen::Matrix<float, Dynamic, Dynamic, Eigen::RowMajor>> depth(image.ptr<float>(), image.rows, image.cols);

    return std::make_pair(true, depth);
}

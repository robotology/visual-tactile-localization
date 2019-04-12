/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RealsenseCamera.h>

#include <yarp/cv/Cv.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>

using namespace Eigen;
using namespace yarp::cv;
using namespace yarp::os;
using namespace yarp::sig;


RealsenseCamera::RealsenseCamera
(
    const std::string& port_prefix,
    const std::string& fallback_context,
    const std::string& fallback_key
)
{
    Property properties;
    properties.put("device", "RGBDSensorClient");
    properties.put("localImagePort",  "/" + port_prefix + "/RGBDSensorClient/image:i");
    properties.put("localDepthPort",  "/" + port_prefix + "/RGBDSensorClient/depth:i");
    properties.put("localRpcPort",    "/" + port_prefix + "/RGBDSensorClient/rpc:i");
    properties.put("remoteImagePort", "/depthCamera/rgbImage:o");
    properties.put("remoteDepthPort", "/depthCamera/depthImage:o");
    properties.put("remoteRpcPort",   "/depthCamera/rpc:i");

    if (rgbd_drv_.open(properties) && rgbd_drv_.view(irgbd_) && (irgbd_ != nullptr))
    {
        yInfo() << log_ID_ << "::ctor. Using driver to connect to the realsense camera.";

        yarp::os::Property camera_intrinsics;
        irgbd_->getRgbIntrinsicParam(camera_intrinsics);

        parameters_.width = irgbd_->getRgbWidth();
        parameters_.height = irgbd_->getRgbHeight();
        parameters_.fx = camera_intrinsics.find("focalLengthX").asFloat64();
        parameters_.fy = camera_intrinsics.find("focalLengthY").asFloat64();
        parameters_.cx = camera_intrinsics.find("principalPointX").asFloat64();
        parameters_.cy = camera_intrinsics.find("principalPointY").asFloat64();
        parameters_.initialized = true;
    }
    else
    {
        yWarning() << log_ID_ << "::ctor. Driver not available, using raw ports.";

        use_drv_ = false;

        // Load camera resolution and instrinsic parameters from configuration file
        ResourceFinder rf;
        rf.setVerbose(true);
        rf.setDefaultContext(fallback_context);
        rf.setDefaultConfigFile("realsense_camera_config.ini");
        rf.configure(0, nullptr);

        ResourceFinder rf_camera = rf.findNestedResourceFinder(fallback_key.c_str());
        bool ok =  rf_camera.check("width");
        ok &= rf_camera.check("height");
        ok &= rf_camera.check("fx");
        ok &= rf_camera.check("fy");
        ok &= rf_camera.check("cx");
        ok &= rf_camera.check("cy");
        if (!ok)
        {
            std::string err = log_ID_ + "::ctor. Error: cannot load realsense camera parameters.";
            throw(std::runtime_error(err));
        }

        parameters_.width = rf_camera.find("width").asDouble();
        parameters_.height = rf_camera.find("height").asDouble();
        parameters_.fx = rf_camera.find("fx").asDouble();
        parameters_.fy = rf_camera.find("fy").asDouble();
        parameters_.cx = rf_camera.find("cx").asDouble();
        parameters_.cy = rf_camera.find("cy").asDouble();
        parameters_.initialized = true;

        // // Open rgb input port
        // if (!(port_rgb_in_.open("/" + port_prefix + "/rgbImage:i")))
        // {
        //     std::string err = log_ID_ + "::ctor. Error: cannot open realsense rgb camera input port.";
        //     throw(std::runtime_error(err));
        // }

        // // Open depth input port
        // if (!(port_depth_in_.open("/" + port_prefix + "/depthImage:i")))
        // {
        //     std::string err = log_ID_ + "::ctor. Error: cannot open realsense depth camera input port.";
        //     throw(std::runtime_error(err));
        // }
    }

    // Log parameters
    std::cout << log_ID_ + "::ctor. Camera parameters." << std::endl;
    std::cout << log_ID_ + "    - width: " << parameters_.width << std::endl;
    std::cout << log_ID_ + "    - height: " << parameters_.height << std::endl;
    std::cout << log_ID_ + "    - fx: " << parameters_.fx << std::endl;
    std::cout << log_ID_ + "    - fy: " << parameters_.fy << std::endl;
    std::cout << log_ID_ + "    - cx: " << parameters_.cx << std::endl;
    std::cout << log_ID_ + "    - cy: " << parameters_.cy << std::endl;

    // Open rgb input port
    if (!(port_rgb_in_.open("/" + port_prefix + "/rgbImage:i")))
    {
        std::string err = log_ID_ + "::ctor. Error: cannot open realsense rgb camera input port.";
        throw(std::runtime_error(err));
    }

    // Open depth input port
    if (!(port_depth_in_.open("/" + port_prefix + "/depthImage:i")))
    {
        std::string err = log_ID_ + "::ctor. Error: cannot open realsense depth camera input port.";
        throw(std::runtime_error(err));
    }
}


RealsenseCamera::~RealsenseCamera()
{
    if (use_drv_)
    {
        rgbd_drv_.close();
    }
    // else
    // {
    port_depth_in_.close();
    port_rgb_in_.close();
    // }
}


std::pair<bool, Transform<double, 3, Affine>> RealsenseCamera::getCameraPose(const bool& blocking)
{
    return std::make_pair(true, Transform<double, 3, Affine>::Identity());
}


std::pair<bool, cv::Mat> RealsenseCamera::getRgbImage(const bool& blocking)
{
    // if (use_drv_)
    // {
    //     // TO BE DONE, it seems that nobody uses RGBDSensorClient.getRgbImage(). Why?
    // }
    // else
    // {
    // Get image
    ImageOf<PixelRgb>* image_in;
    image_in = port_rgb_in_.read(blocking);

    if (image_in == nullptr)
        return std::make_pair(false, cv::Mat());

    cv::Mat image = yarp::cv::toCvMat(*image_in);
    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

    return std::make_pair(true, image);
    // }
}


std::pair<bool, Eigen::MatrixXf> RealsenseCamera::getDepthImage(const bool& blocking)
{
    // if (use_drv_)
    // {
    //     // TO BE DONE, it seems that nobody uses RGBDSensorClient.getDepthImage(). Why?
    // }
    // else
    // {
    ImageOf<PixelFloat>* image_in;
    image_in = port_depth_in_.read(blocking);

    if (image_in == nullptr)
        return std::make_pair(false, MatrixXf());

    cv::Mat image = yarp::cv::toCvMat(*image_in);
    Map<Eigen::Matrix<float, Dynamic, Dynamic, Eigen::RowMajor>> depth(image.ptr<float>(), image.rows, image.cols);

    return std::make_pair(true, depth);
    // }
}

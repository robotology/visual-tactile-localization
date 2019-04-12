/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef REALSENSE_H
#define REALSENSE_H

#include <Camera.h>

#include <Eigen/Dense>

#include <yarp/dev/IRGBDSensor.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Image.h>

class RealsenseCamera : public Camera
{
public:

    RealsenseCamera(const std::string& port_prefix, const std::string& fallback_configuration, const std::string& fallback_key);

    ~RealsenseCamera();

    std::pair<bool, Eigen::Transform<double, 3, Eigen::Affine>> getCameraPose(const bool& blocking) override;

    std::pair<bool, cv::Mat> getRgbImage(const bool& blocking) override;

    std::pair<bool, Eigen::MatrixXf> getDepthImage(const bool& blocking) override;

protected:

    const std::string log_ID_ = "RealsenseCamera";

    bool use_drv_ = true;

    yarp::dev::PolyDriver rgbd_drv_;

    yarp::dev::IRGBDSensor* irgbd_;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat>> port_depth_in_;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> port_rgb_in_;
};

#endif /* REALSENSE_H */

/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ICUBCAMERA_H
#define ICUBCAMERA_H

#include <Camera.h>

#include <Eigen/Dense>

#include <iCub/iKin/iKinFwd.h>

#include <opencv2/opencv.hpp>

#include <string>

#include <yarp/dev/GazeControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Image.h>


class iCubCamera : public Camera
{
public:

    iCubCamera(const std::string& laterality, const std::string& port_prefix, const std::string& fallback_context, const std::string& fallback_key);

    ~iCubCamera();

    std::pair<bool, Eigen::Transform<double, 3, Eigen::Affine>> getCameraPose(const bool& blocking) override;

    std::pair<bool, cv::Mat> getRgbImage(const bool& blocking) override;

    std::pair<bool, Eigen::MatrixXf> getDepthImage(const bool& blocking) override;

protected:

    const std::string log_ID_ = "iCubCamera";

    const std::string laterality_;

    bool use_igaze_ = true;

    bool depth_available = false;

    yarp::dev::PolyDriver drv_gaze_;

    yarp::dev::IGazeControl* igaze_;

    yarp::os::BufferedPort<yarp::os::Bottle> port_head_enc_;

    yarp::os::BufferedPort<yarp::os::Bottle> port_torso_enc_;

    iCub::iKin::iCubEye icub_eye_fk_;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat>> port_depth_in_;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> port_rgb_in_;
};

#endif /* ICUBCAMERA_H */

/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef YCBVIDEOCAMERA_H
#define YCBVIDEOCAMERA_H

#include <Camera.h>

#include <Eigen/Dense>

#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Image.h>


class YcbVideoCamera : public Camera
{
public:

    YcbVideoCamera(const std::string& port_prefix, const std::string& fallback_context, const std::string& fallback_key);

    ~YcbVideoCamera();

    std::pair<bool, Eigen::Transform<double, 3, Eigen::Affine>> getCameraPose(const bool& blocking) override;

    std::pair<bool, cv::Mat> getRgbImage(const bool& blocking) override;

    std::pair<bool, Eigen::MatrixXf> getDepthImage(const bool& blocking) override;

protected:

    const std::string log_ID_ = "YcbVideoCamera";

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat>> port_depth_in_;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> port_rgb_in_;
};

#endif /* YCBVIDEOCAMERA_H */

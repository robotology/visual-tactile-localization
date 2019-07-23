/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef VALIDATOR2D_H
#define VALIDATOR2D_H

#include <Eigen/Dense>

#include <Camera.h>
#include <PointCloudPrediction.h>

#include <memory>

#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Image.h>


class Validator2D
{
public:
    Validator2D(std::shared_ptr<PointCloudPrediction> point_cloud_prediction, const std::string& port_prefix, const std::string& camera_name, const std::string& camera_fallback_key, const std::string& camera_laterality = "");

    Validator2D(std::shared_ptr<PointCloudPrediction> point_cloud_prediction, const std::string& port_prefix, const std::string& camera_path);

    ~Validator2D();

    bool reset();

    bool renderEvaluation(const Eigen::Transform<double, 3, Eigen::Affine> object_pose);

private:
    std::unique_ptr<Camera> camera_;

    std::shared_ptr<PointCloudPrediction> prediction_;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> port_image_in_;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> port_image_out_;

    cv::Mat image_in_;

    const std::string log_ID_ = "Validator2D";
};

#endif /* VALIDATOR2D_H */

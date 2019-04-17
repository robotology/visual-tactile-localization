/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef BOUNDINGBOXSEGMENTATION_H
#define BOUNDINGBOXSEGMENTATION_H

#include <Eigen/Dense>

#include <PointCloudSegmentation.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Image.h>


class BoundingBoxSegmentation : public PointCloudSegmentation
{
public:

    BoundingBoxSegmentation(const std::string& port_prefix, const Eigen::Ref<const Eigen::VectorXd>& bounding_box, const std::size_t& depth_stride);

    ~BoundingBoxSegmentation();

    bool freezeSegmentation(Camera& camera) override;

    std::pair<bool, Eigen::MatrixXd> extractPointCloud(Camera& camera, const Eigen::Ref<const Eigen::MatrixXf>& depth, const double& max_depth = 1.0) override;

    bool getProperty(const std::string& property) const override;

    bool setProperty(const std::string& property) override;

protected:

    void drawBoundingBoxOnCamera(const cv::Mat& bounding_box_mask, Camera& camera);

    Eigen::Vector4d bounding_box_;

    double depth_stride_;

    Eigen::Transform<double, 3, Eigen::Affine> camera_pose_;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> port_image_out_;

    std::vector<std::pair<int, int>> coordinates_;

    const std::string log_ID_ = "BoundingBoxSegmentation";
};

#endif /* BOUNDINGBOXSEGMENTATION_H */

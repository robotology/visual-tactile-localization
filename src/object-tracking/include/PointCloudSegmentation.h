/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef POINTCLOUDSEGMENTATION_H
#define POINTCLOUDSEGMENTATION_H

#include <Camera.h>
#include <ObjectOcclusion.h>

#include <Eigen/Dense>

#include <string>


class PointCloudSegmentation
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PointCloudSegmentation();

    virtual ~PointCloudSegmentation();

    virtual bool freezeSegmentation(Camera& camera) = 0;

    virtual std::pair<bool, Eigen::MatrixXd> extractPointCloud(Camera& camera, const Eigen::Ref<const Eigen::MatrixXf>& depth, const double& max_depth) = 0;

    virtual std::pair<bool, Eigen::MatrixXd> extractSegmentation();

    virtual void addObjectOcclusion(std::unique_ptr<ObjectOcclusion> object_occlusion);

    virtual bool getProperty(const std::string& property) const;

    virtual std::size_t getDepthStride() const;

    virtual void setObjectPose(const Eigen::Transform<double, 3, Eigen::Affine>& pose);

    virtual bool setProperty(const std::string& property);

    virtual void setDepthStride(const std::size_t& stride);

    virtual void setDefaultDepthStride(const std::size_t& stride);

    virtual void reset();

protected:

    /**
     * Object occlusions.
     */
    std::vector<std::unique_ptr<ObjectOcclusion>> occlusions_;

    /**
     * Object pose.
     */
    Eigen::Transform<double, 3, Eigen::Affine> object_pose_;

    bool object_pose_available_ = false;

private:

    std::size_t depth_stride_;

    std::size_t default_depth_stride_;

    const std::string log_ID_ = "[PointCloudSegmentation]";
};

#endif /* POINTCLOUDSEGMENTATION_H */

/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <PointCloudSegmentation.h>


PointCloudSegmentation::PointCloudSegmentation()
{ }


PointCloudSegmentation::~PointCloudSegmentation()
{ }


void PointCloudSegmentation::addObjectOcclusion(std::unique_ptr<ObjectOcclusion> object_occlusion)
{
    occlusions_.push_back(std::move(object_occlusion));
}


bool PointCloudSegmentation::getProperty(const std::string& property) const
{
    return false;
}


std::size_t PointCloudSegmentation::getDepthStride() const
{
    return depth_stride_;
}


void PointCloudSegmentation::setObjectPose(const Eigen::Transform<double, 3, Eigen::Affine>& pose)
{
    object_pose_ = pose;
    object_pose_available_ = true;
}


bool PointCloudSegmentation::setProperty(const std::string& property)
{
    if (property == "reset")
    {
        reset();

        return true;
    }

    return false;
}


void PointCloudSegmentation::setDepthStride(const std::size_t& stride)
{
    depth_stride_ = stride;
}


void PointCloudSegmentation::setDefaultDepthStride(const std::size_t& stride)
{
    default_depth_stride_ = stride;
}


void PointCloudSegmentation::reset()
{
    object_pose_available_ = false;

    setDepthStride(default_depth_stride_);

    for (auto& occlusion : occlusions_)
        occlusion->reset();
}

/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <MaskSegmentation.h>


MaskSegmentation::MaskSegmentation(const std::string& port_prefix, const std::size_t& depth_stride)
{

}


MaskSegmentation::~MaskSegmentation()
{ }


bool MaskSegmentation::freezeSegmentation(Camera& camera)
{

}


std::pair<bool, Eigen::MatrixXd> MaskSegmentation::extractPointCloud(Camera& camera, const Eigen::Ref<const Eigen::MatrixXf>& depth, const double& max_depth)
{

}


bool MaskSegmentation::getProperty(const std::string& property) const
{
    return true;
}


bool MaskSegmentation::setProperty(const std::string& property)
{
    return true;
}

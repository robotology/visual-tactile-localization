/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef MASKSEGMENTATION_H
#define MASKSEGMENTATION_H

#include <PointCloudSegmentation.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Image.h>


class MaskSegmentation : public PointCloudSegmentation
{
public:

    MaskSegmentation(const std::string& port_prefix, const std::size_t& depth_stride);

    ~MaskSegmentation();

    bool freezeSegmentation(Camera& camera) override;

    std::pair<bool, Eigen::MatrixXd> extractPointCloud(Camera& camera, const Eigen::Ref<const Eigen::MatrixXf>& depth, const double& max_depth = 1.0) override;

    bool getProperty(const std::string& property) const override;

    bool setProperty(const std::string& property) override;

protected:


};

#endif /* MASKSEGMENTATION_H */

/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef POINTCLOUDDUMPER_H
#define POINTCLOUDDUMPER_H

#include <BayesFilters/AdditiveMeasurementModel.h>
#include <BayesFilters/Data.h>

#include <Camera.h>
#include <PointCloudSegmentation.h>

#include <Eigen/Dense>

#include <memory>

#include <yarp/os/Port.h>

#include <thrift/PointCloudDumperIDL.h>


class PointCloudDumper : public bfl::AdditiveMeasurementModel,
                         public PointCloudDumperIDL
{
public:
    PointCloudDumper(std::unique_ptr<Camera> camera, std::shared_ptr<PointCloudSegmentation> segmentation, const std::string& depth_fetch_mode, const std::string& port_prefix);

    virtual ~PointCloudDumper();

    std::pair<bool, bfl::Data> measure(const bfl::Data& data = bfl::Data()) const;

    bool freeze() override;

    std::pair<std::size_t, std::size_t> getOutputSize() const;

    std::pair<bool, bfl::Data> predictedMeasure(const Eigen::Ref<const Eigen::MatrixXd>& current_states) const override;

    std::pair<bool, bfl::Data> innovation(const bfl::Data& predicted_measurements, const bfl::Data& measurements) const override;

protected:

    bool getDepth();

    bool save_frames();

    bool reset();

    /**
     * Local copy of depht image.
     */
    Eigen::MatrixXf depth_;

    std::string depth_fetch_mode_;

    bool depth_initialized_ = false;

    /**
     * Local copy of measurements.
     * A vector of size 3 * L with L the number of points in set.
     */
    std::vector<Eigen::MatrixXd> measurement_;

    /**
     * Size of the last data
     */
    std::unique_ptr<Camera> camera_;

    std::shared_ptr<PointCloudSegmentation> segmentation_;

    yarp::os::Port port_rpc_command_;
};

#endif /* POINTCLOUDDUMPER_H */

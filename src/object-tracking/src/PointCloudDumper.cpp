/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <PointCloudDumper.h>

#include <fstream>

using namespace bfl;
using namespace Eigen;


PointCloudDumper::PointCloudDumper
(
    std::unique_ptr<Camera> camera,
    std::shared_ptr<PointCloudSegmentation> segmentation,
    const std::string& depth_fetch_mode,
    const std::string& port_prefix
) :
    camera_(std::move(camera)),
    segmentation_(segmentation),
    depth_fetch_mode_(depth_fetch_mode)
{
    // Open RPC input port for commands
    if (!port_rpc_command_.open("/" + port_prefix + "/point-cloud-dumper/cmd:i"))
    {
        std::string err = "POINTCLOUDDUMPER::CTOR::ERROR\n\tError: cannot open RPC command port.";
        throw(std::runtime_error(err));
    }

    if (!(this->yarp().attachAsServer(port_rpc_command_)))
    {
        std::string err = "POINTCLOUDDUMPER::CTOR::ERROR\n\tError: cannot attach the RPC command port.";
        throw(std::runtime_error(err));
    }
}


PointCloudDumper::~PointCloudDumper()
{
    port_rpc_command_.close();
}


std::pair<bool, Data> PointCloudDumper::measure(const Data& data) const
{
    return std::make_pair(false, Data());
}


bool PointCloudDumper::freeze()
{
    // Freeze segmentation
    if (!segmentation_->freezeSegmentation(*camera_))
        return false;

    // Get depth image
    if(!getDepth())
        return false;

    // Get 3D point cloud.
    bool blocking_call = false;
    bool valid_point_cloud;
    MatrixXd point_cloud;
    std::tie(valid_point_cloud, point_cloud) = segmentation_->extractPointCloud(*camera_, depth_, 10.0);
    if (!valid_point_cloud)
        return false;

    // Save frame
    measurement_.push_back(point_cloud);

    // Return false as we do not want to perform actual filtering
    return false;
}


std::pair<std::size_t, std::size_t> PointCloudDumper::getOutputSize() const
{
    return std::make_pair(0, 0);
}


std::pair<bool, Data> PointCloudDumper::predictedMeasure(const Eigen::Ref<const Eigen::MatrixXd>& cur_states) const
{
    return std::make_pair(false, Data());
}


std::pair<bool, Data> PointCloudDumper::innovation(const Data& predicted_measurements, const Data& measurements) const
{
    return std::make_pair(false, Data());
}


bool PointCloudDumper::getDepth()
{
    std::string mode = depth_fetch_mode_;
    if (!depth_initialized_)
    {
        // in case a depth was never received
        // it is required to wait at least for the first image in blocking mode
        mode = "new_image";
    }

    bool valid_depth;
    MatrixXf tmp_depth;
    std::tie(valid_depth, tmp_depth) = camera_->getDepthImage(mode == "new_image");

    if (valid_depth)
    {
        depth_ = std::move(tmp_depth);
        depth_initialized_ = true;
    }

    if (mode == "skip")
        return depth_initialized_ && valid_depth;
    else
        return depth_initialized_;
}


bool PointCloudDumper::save_frames()
{
    std::ofstream out;

    for (std::size_t i = 0; i < measurement_.size(); i++)
    {
        const Ref<MatrixXd> point_cloud = measurement_.at(i);
        out.open("frame_" + std::to_string(i) + ".OFF");

        if (!out.is_open())
        {
            std::cout << "Unable to open file for frame " << i << std::endl;
            return false;
        }

        out << "OFF" << std::endl;
        out << point_cloud.cols() << " 0 0" << std::endl;

        for (std::size_t j = 0; j < point_cloud.cols(); j++)
        {
            out << point_cloud.col(j).transpose() << std::endl;
        }

        out.close();

        std::cout << "Saved frame " << i << std::endl;
    }

    std::cout << "Frames saved succesfully." << std::endl;

    return true;
}


bool PointCloudDumper::reset()
{
    measurement_.clear();

    return true;
}

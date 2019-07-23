/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ObjectMeasurements.h>

using namespace bfl;
using namespace Eigen;


ObjectMeasurements::ObjectMeasurements
(
    std::unique_ptr<Camera> camera,
    std::shared_ptr<PointCloudSegmentation> segmentation,
    std::shared_ptr<PointCloudPrediction> prediction,
    const Eigen::Ref<const Eigen::Matrix3d>& visual_noise_covariance,
    const double& visual_outlier_threshold,
    const std::string& depth_fetch_mode,
    const bool& enable_log,
    const std::string& log_path
) :
    camera_(std::move(camera)),
    segmentation_(segmentation),
    prediction_(prediction),
    visual_noise_covariance_(visual_noise_covariance),
    visual_outlier_threshold_(visual_outlier_threshold),
    depth_fetch_mode_(depth_fetch_mode),
    enable_log_(enable_log),
    log_path_(log_path)
{ }


ObjectMeasurements::ObjectMeasurements
(
    std::unique_ptr<Camera> camera,
    std::shared_ptr<PointCloudSegmentation> segmentation,
    std::shared_ptr<PointCloudPrediction> prediction,
    const Eigen::Ref<const Eigen::Matrix3d>& visual_noise_covariance,
    const Eigen::Ref<const Eigen::Matrix3d>& tactile_noise_covariance,
    const double& visual_outlier_threshold,
    const std::string& depth_fetch_mode,
    const bool& enable_log,
    const std::string& log_path
) :
    ObjectMeasurements(std::move(camera), segmentation, prediction, visual_noise_covariance, visual_outlier_threshold, depth_fetch_mode, enable_log, log_path)
{
    tactile_noise_covariance_ = tactile_noise_covariance;
    has_tactile_noise_covariance_ = true;
}


std::pair<bool, Data> ObjectMeasurements::measure(const Data& data) const
{
    return std::make_pair(true, measurement_);
}


bool ObjectMeasurements::freeze()
{
    // Freeze camera
    measurements_available_ = camera_->freeze();
    if (!measurements_available_)
        return false;

    // Get depth image
    if(!getDepth())
        return false;

    // Freeze segmentation
    if (!segmentation_->freezeSegmentation(*camera_))
        return false;

    // Get 3D point cloud.
    bool blocking_call = false;
    bool valid_point_cloud;
    MatrixXd point_cloud;
    std::tie(valid_point_cloud, point_cloud) = segmentation_->extractPointCloud(*camera_, depth_);
    if (!valid_point_cloud)
        return false;

    // Evaluate centroid of point cloud and discard too far points
    VectorXd centroid = (point_cloud.rowwise().sum()) / point_cloud.cols();
    VectorXi good_points(point_cloud.cols());
    for (int i = 0 ; i < point_cloud.cols(); i++)
    {
        good_points(i) = 0;
        if ((point_cloud.col(i) - centroid).norm() < visual_outlier_threshold_)
            good_points(i) = 1;
    }
    visual_data_size_ = good_points.sum();

    // Take only valid visual points
    MatrixXd points(3, visual_data_size_);
    for (int i = 0, j = 0; i < point_cloud.cols(); i++)
    {
        if (good_points(i) == 1)
        {
            point_cloud.col(i).swap(points.col(j));
            j++;
        }
    }

    // Resize measurements to be a column vector.
    measurement_.resize(3 * points.cols(), 1);
    measurement_.swap(Map<MatrixXd>(points.data(), points.size(), 1));

    // Log measurements
    logger(measurement_.transpose());

    return true;
}


std::pair<std::size_t, std::size_t> ObjectMeasurements::getOutputSize() const
{
    return std::make_pair(measurement_.size(), 0);
}


std::pair<bool, bfl::Data> ObjectMeasurements::predictedMeasure(const Eigen::Ref<const Eigen::MatrixXd>& cur_states) const
{
    bool valid_measure;
    bfl::Data measurement;
    std::tie(valid_measure, measurement) = measure();

    if (!valid_measure)
        return std::make_pair(false, Data());

    bool valid_prediction;
    MatrixXd prediction;
    std::tie(valid_prediction, prediction) = prediction_->predictPointCloud(cur_states, any::any_cast<MatrixXd>(measurement).col(0));

    if (!valid_prediction)
        return std::make_pair(false, Data());

    return std::make_pair(true, prediction);
}


std::pair<bool, bfl::Data> ObjectMeasurements::innovation(const bfl::Data& predicted_measurements, const bfl::Data& measurements) const
{
    MatrixXd innovation = -(any::any_cast<MatrixXd>(predicted_measurements).colwise() - any::any_cast<MatrixXd>(measurements).col(0));

    return std::make_pair(true, std::move(innovation));
}


std::pair<bool, Eigen::MatrixXd> ObjectMeasurements::getNoiseCovarianceMatrix() const
{
    std::string err = "ObjectMeasurements::ctor. Error: method not implemented.";
    throw(std::runtime_error(err));
}


std::pair<bool, MatrixXd> ObjectMeasurements::getVisualNoiseCovarianceMatrix() const
{
    return std::make_pair(true, visual_noise_covariance_);
}


std::pair<bool, MatrixXd> ObjectMeasurements::getTactileNoiseCovarianceMatrix() const
{
    if (!has_tactile_noise_covariance_)
        return std::make_pair(false, MatrixXd());

    return std::make_pair(true, tactile_noise_covariance_);
}


std::size_t ObjectMeasurements::getVisualDataSize() const
{
    return visual_data_size_;
}


std::size_t ObjectMeasurements::getTactileDataSize() const
{
    return tactile_data_size_;
}


bool ObjectMeasurements::setProperty(const std::string& property)
{
    if (property == "reset")
    {
        reset();

        return true;
    }
    else if(property == "enable_log")
    {
        if (enable_log_)
            enable_log(log_path_, "object-tracking");
    }
    else if (property == "measurements_available")
    {
        return measurements_available_;
    }

    return false;
}


bool ObjectMeasurements::getDepth()
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


void ObjectMeasurements::reset()
{
    depth_initialized_ = false;

    measurements_available_ = true;

    camera_->reset();

    segmentation_->setProperty("reset");

    disable_log();
}

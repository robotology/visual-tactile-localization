/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <iCubObjectMeasurements.h>

#include <Eigen/Dense>

#include <iostream>

using namespace bfl;
using namespace Eigen;


iCubObjectMeasurements::iCubObjectMeasurements
(
    std::unique_ptr<Camera> camera,
    std::shared_ptr<PointCloudSegmentation> segmentation,
    std::unique_ptr<PointCloudPrediction> prediction,
    const Ref<const Matrix3d>& visual_noise_covariance,
    const double& visual_outlier_threshold,
    const std::string& depth_fetch_mode
) :
    ObjectMeasurements(std::move(camera), segmentation, std::move(prediction), visual_noise_covariance, visual_outlier_threshold, depth_fetch_mode)
{ }


iCubObjectMeasurements::iCubObjectMeasurements
(
    std::unique_ptr<Camera> camera,
    std::shared_ptr<PointCloudSegmentation> segmentation,
    std::unique_ptr<PointCloudPrediction> prediction,
    std::unique_ptr<iCubHandContactsModel> object_contacts,
    const Ref<const Matrix3d>& visual_noise_covariance,
    const Ref<const Matrix3d>& tactile_noise_covariance,
    const double& visual_outlier_threshold,
    const std::string& depth_fetch_mode
) :
    ObjectMeasurements(std::move(camera), segmentation, std::move(prediction), visual_noise_covariance, tactile_noise_covariance, visual_outlier_threshold, depth_fetch_mode),
    contacts_(std::move(object_contacts))
{ }


iCubObjectMeasurements::~iCubObjectMeasurements()
{ }


std::pair<bool, Data> iCubObjectMeasurements::measure() const
{
    return std::make_pair(true, measurement_);
}


bool iCubObjectMeasurements::freezeMeasurements()
{
    // Reset number of contact points
    tactile_data_size_ = 0;

    // Check if contact is occurring
    VectorXd tactile_points;
    if ((contacts_ != nullptr) && (contacts_->freezeMeasurements()))
    {
        tactile_points = contacts_->measure();

        tactile_data_size_ = tactile_points.size() / 3;
    }

    // If contact is occuring, the segmentation can latch the bounding box to the hand motion
    if (tactile_data_size_ > 0)
        segmentation_->setProperty("latch_to_hand");

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

    // Allocate storage
    std::size_t total_number_points = good_points.sum();
    if (use_contacts_)
        total_number_points += tactile_data_size_;
    MatrixXd points(3, total_number_points);

    // Take only valid visual points
    int j = 0;
    for (int i = 0; i < point_cloud.cols(); i++)
    {
        if (good_points(i) == 1)
        {
            point_cloud.col(i).swap(points.col(j));
            j++;
        }
    }

    // If the user requested them and if they are available, take also contact points
    if (use_contacts_)
    {
        for (int i = 0; i < tactile_data_size_; i++)
        {
            tactile_points.segment(i * 3, 3).swap(points.col(j));

            j++;
        }
    }

    // resize measurements to be a column vector.
    measurement_.resize(3 * points.cols(), 1);
    measurement_.swap(Map<MatrixXd>(points.data(), points.size(), 1));

    // Log measurements
    logger(measurement_.transpose());

    // Notify the user that contact points are used
    if ((use_contacts_) && (tactile_data_size_ > 0))
        std::cout << "Info: using contacts." << std::endl << std::flush;

    return true;
}


std::pair<bool, Eigen::MatrixXd> iCubObjectMeasurements::getNoiseCovarianceMatrix() const
{
    if (contacts_ == nullptr)
    {
        return std::make_pair(true, visual_noise_covariance_);
    }

    std::string err = log_ID_ + "::getNoiseCovarianceMatrix. Error: since support for tactile measurements is active, you should use methods getVisualNoiseCovarianceMatrix() and getTactileNoiseCovarianceMatrix() instead of getNoiseCovarianceMatrix().";
    throw(std::runtime_error(err));
}


bool iCubObjectMeasurements::setProperty(const std::string& property)
{
    bool outcome = ObjectMeasurements::setProperty(property);

    if (property == "use_contacts_on")
    {
        use_contacts_ = true;
    }
    else if (property == "use_contacts_off")
    {
        use_contacts_ = false;
    }
    else if (property == "get_contact_state")
    {
        return (tactile_data_size_ > 0);
    }
    else
        outcome = false;

    return outcome;
}


void iCubObjectMeasurements::reset()
{
    ObjectMeasurements::reset();

    use_contacts_ = true;
}

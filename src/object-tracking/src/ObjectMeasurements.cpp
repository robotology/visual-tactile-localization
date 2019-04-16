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
    std::unique_ptr<PointCloudPrediction> prediction,
    const Eigen::Ref<const Eigen::Matrix3d>& visual_noise_covariance
) :
    camera_(std::move(camera)),
    segmentation_(segmentation),
    prediction_(std::move(prediction)),
    visual_noise_covariance_(visual_noise_covariance)
{ }


ObjectMeasurements::ObjectMeasurements
(
    std::unique_ptr<Camera> camera,
    std::shared_ptr<PointCloudSegmentation> segmentation,
    std::unique_ptr<PointCloudPrediction> prediction,
    const Eigen::Ref<const Eigen::Matrix3d>& visual_noise_covariance,
    const Eigen::Ref<const Eigen::Matrix3d>& tactile_noise_covariance
) :
    ObjectMeasurements(std::move(camera), segmentation, std::move(prediction), visual_noise_covariance)
{
    tactile_noise_covariance_ = tactile_noise_covariance;
    has_tactile_noise_covariance_ = true;
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
    return 0;
}


std::size_t ObjectMeasurements::getTactileDataSize() const
{
    return 0;
}


bool ObjectMeasurements::setProperty(const std::string& property)
{
    if (property == "reset")
    {
        reset();

        return true;
    }

    return false;
}


void ObjectMeasurements::reset()
{
    segmentation_->setProperty("reset");
}

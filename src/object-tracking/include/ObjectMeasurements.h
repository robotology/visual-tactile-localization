/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef OBJECTMEASUREMENTS_H
#define OBJECTMEASUREMENTS_H

#include <BayesFilters/AdditiveMeasurementModel.h>

#include <Camera.h>
#include <PointCloudPrediction.h>
#include <PointCloudSegmentation.h>

#include <Eigen/Dense>

#include <memory>


class ObjectMeasurements : public bfl::AdditiveMeasurementModel
{
public:
    ObjectMeasurements(std::unique_ptr<Camera> camera, std::shared_ptr<PointCloudSegmentation> segmentation, std::unique_ptr<PointCloudPrediction> prediction, const Eigen::Ref<const Eigen::Matrix3d>& visual_noise_covariance);

    ObjectMeasurements(std::unique_ptr<Camera> camera, std::shared_ptr<PointCloudSegmentation> segmentation, std::unique_ptr<PointCloudPrediction> prediction, const Eigen::Ref<const Eigen::Matrix3d>& visual_noise_covariance, const Eigen::Ref<const Eigen::Matrix3d>& tactile_noise_covariance);

    std::pair<bool, bfl::Data> predictedMeasure(const Eigen::Ref<const Eigen::MatrixXd>& current_states) const override;

    std::pair<bool, bfl::Data> innovation(const bfl::Data& predicted_measurements, const bfl::Data& measurements) const override;

    std::pair<bool, Eigen::MatrixXd> getNoiseCovarianceMatrix() const override;

    std::pair<bool, Eigen::MatrixXd> getVisualNoiseCovarianceMatrix() const;

    std::pair<bool, Eigen::MatrixXd> getTactileNoiseCovarianceMatrix() const;

    virtual std::size_t getVisualDataSize() const;

    virtual std::size_t getTactileDataSize() const;

    bool setProperty(const std::string& property) override;

protected:

    virtual void reset();

    std::unique_ptr<Camera> camera_;

    std::shared_ptr<PointCloudSegmentation> segmentation_;

    std::unique_ptr<PointCloudPrediction> prediction_;

    Eigen::Matrix3d visual_noise_covariance_;

    Eigen::Matrix3d tactile_noise_covariance_;

    bool has_tactile_noise_covariance_ = false;

    std::vector<std::string> log_file_names(const std::string& prefix_path, const std::string& prefix_name) override
    {
        return {prefix_path + "/" + prefix_name + "_measurements"};
    }
};

#endif /* OBJECTMEASUREMENTS_H */

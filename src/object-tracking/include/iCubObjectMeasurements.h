/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ICUBOBJECTMEASUREMENTS_H
#define ICUBOBJECTMEASUREMENTS_H

#include <BayesFilters/Data.h>

#include <Eigen/Dense>

#include <ObjectMeasurements.h>
#include <iCubHandContactsModel.h>

#include <memory>
#include <string>


class iCubObjectMeasurements : public ObjectMeasurements
{
public:
    iCubObjectMeasurements(std::unique_ptr<Camera> camera, std::shared_ptr<PointCloudSegmentation>, std::unique_ptr<PointCloudPrediction> prediction, const Eigen::Ref<const Eigen::Matrix3d>& visual_noise_covariance, const double& visual_outlier_threshold, const std::string& depth_fetch_mode);

    iCubObjectMeasurements(std::unique_ptr<Camera> camera, std::shared_ptr<PointCloudSegmentation>, std::unique_ptr<PointCloudPrediction> prediction, std::unique_ptr<iCubHandContactsModel> object_contacts, const Eigen::Ref<const Eigen::Matrix3d>& visual_noise_covariance, const Eigen::Ref<const Eigen::Matrix3d>& tactile_noise_covariance, const double& visual_outlier_threshold, const std::string& depth_fetch_mode);

    virtual ~iCubObjectMeasurements();

    std::pair<bool, bfl::Data> measure() const override;

    bool freezeMeasurements() override;

    std::pair<std::size_t, std::size_t> getOutputSize() const override;

    std::pair<bool, Eigen::MatrixXd> getNoiseCovarianceMatrix() const override;

    std::size_t getVisualDataSize() const override;

    std::size_t getTactileDataSize() const override;

    bool setProperty(const std::string& property) override;

protected:

    void reset() override;

    /**
     * Depth.
     */
    bool getDepth();

    Eigen::MatrixXf depth_;

    std::string depth_fetch_mode_;

    bool depth_initialized_ = false;

    /**
     * iCub hand contacts
     */
    std::unique_ptr<iCubHandContactsModel> contacts_;

    bool use_contacts_ = true;

    /**
     * Local copy of measurements.
     */
    Eigen::MatrixXd measurement_;

    /**
     * Threshold for detection of outliers in visual data.
     */
    double visual_outlier_threshold_;

    /**
     * Size of the last data
     */
    std::size_t visual_data_size_ = 0;

    std::size_t tactile_data_size_ = 0;

    const std::string log_ID_ = "iCubObjectMeasurements";
};

#endif /* ICUBOBJECTMEASUREMENTS_H */

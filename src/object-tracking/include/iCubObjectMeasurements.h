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

    std::pair<bool, bfl::Data> measure(const bfl::Data& data = bfl::Data()) const override;

    bool freeze() override;

    std::pair<bool, Eigen::MatrixXd> getNoiseCovarianceMatrix() const override;

    bool setProperty(const std::string& property) override;

protected:

    void reset() override;

    /**
     * iCub hand contacts
     */
    std::unique_ptr<iCubHandContactsModel> contacts_;

    bool use_contacts_ = true;

    const std::string log_ID_ = "iCubObjectMeasurements";
};

#endif /* ICUBOBJECTMEASUREMENTS_H */

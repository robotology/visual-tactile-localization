/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef PROXIMITYLIKELIHOOD_H
#define PROXIMITYLIKELIHOOD_H

#include <BayesFilters/LikelihoodModel.h>
#include <BayesFilters/MeasurementModel.h>

#include <PointCloudPrediction.h>

#include <Eigen/Dense>

#include <memory>

class ProximityLikelihood : bfl::LikelihoodModel
{
public:
    ProximityLikelihood(const double noise_variance, std::shared_ptr<PointCloudPrediction> squared_distance_estimator_);

    virtual ~ProximityLikelihood();

    std::pair<bool, Eigen::VectorXd> likelihood(const bfl::MeasurementModel& measurement_model, const Eigen::Ref<const Eigen::MatrixXd>& pred_states) override;

protected:
    const double gain_;

    std::shared_ptr<PointCloudPrediction> squared_distance_estimator_;
};

#endif /* PROXIMITYMODEL_H */

#ifndef PROXIMITYLIKELIHOOD_H
#define PROXIMITYLIKELIHOOD_H

#include <BayesFilters/LikelihoodModel.h>
#include <BayesFilters/MeasurementModel.h>

#include <NanoflannPointCloudPrediction.h>

#include <Eigen/Dense>

#include <memory>

class ProximityLikelihood : bfl::LikelihoodModel
{
public:
    ProximityLikelihood(const double noise_variance, std::unique_ptr<NanoflannPointCloudPrediction> squared_distance_estimator_);

    virtual ~ProximityLikelihood();

    std::pair<bool, Eigen::VectorXd> likelihood(const bfl::MeasurementModel& measurement_model, const Eigen::Ref<const Eigen::MatrixXd>& pred_states) override;

protected:
    const double gain_;

    std::unique_ptr<NanoflannPointCloudPrediction> squared_distance_estimator_;
};

#endif /* PROXIMITYMODEL_H */

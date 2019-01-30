#include <ProximityLikelihood.h>

using namespace bfl;
using namespace Eigen;


ProximityLikelihood::ProximityLikelihood(const double noise_variance, std::unique_ptr<NanoflannPointCloudPrediction> squared_distance_estimator) :
    gain_(-0.5 / noise_variance),
    squared_distance_estimator_(std::move(squared_distance_estimator))
{ }


ProximityLikelihood::~ProximityLikelihood()
{ }


std::pair<bool, Eigen::VectorXd> ProximityLikelihood::likelihood(const MeasurementModel& measurement_model, const Eigen::Ref<const Eigen::MatrixXd>& pred_states)
{
    // Take the current measurements, e.g. point cloud
    bool valid_measurements;
    Data data_measurements;
    std::tie(valid_measurements, data_measurements) = measurement_model.measure();

    MatrixXd measurements;
    if (valid_measurements)
        measurements = any::any_cast<MatrixXd&&>(std::move(data_measurements));
    else
        return std::make_pair(false, VectorXd::Zero(1));

    // Approximate the distance between the point cloud and each particle in pred_states
    bool valid_distances;
    MatrixXd squared_distances;
    std::tie(valid_distances, squared_distances) = squared_distance_estimator_->evalDistances(pred_states, measurements);

    // Eval likelihood in log space
    VectorXd likelihood(pred_states.cols());
    likelihood = squared_distances.rowwise().sum() * gain_;

    return std::make_pair(true, likelihood);
}

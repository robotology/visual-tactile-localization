#include <Correction.h>

#include <cmath>

using namespace bfl;


MeasurementModelReference::MeasurementModelReference(AdditiveMeasurementModel& measurement_model) :
    meas_model_(dynamic_cast<iCubPointCloud&>(measurement_model))
{ }

iCubPointCloud& MeasurementModelReference::getPointCloudModel()
{
    return meas_model_;
}

Correction::Correction
(
    std::unique_ptr<AdditiveMeasurementModel> meas_model,
    /**
     * Unscented transform parameters
     */
    const std::size_t state_size,
    const double      alpha,
    const double      beta,
    const double      kappa,
    /**
     * Subsize, J, of measurement vector y in R^M such that
     * M = k * J for some positive integer k
     */
    const std::size_t meas_sub_size
) noexcept :
    MeasurementModelReference(*meas_model),
    SUKFCorrection(std::move(meas_model), state_size, alpha, beta, kappa, meas_sub_size, true)
{ }


void Correction::correctStep(const GaussianMixture& pred_state, GaussianMixture& corr_state)
{
    SUKFCorrection::correctStep(pred_state, corr_state);

    // Handle angular components of the state
    corr_state.mean().bottomRows(3) = (std::complex<double>(0.0,1.0) * corr_state.mean().bottomRows(3)).array().exp().arg();
}


void Correction::reset()
{
    getPointCloudModel().setProperty("reset");
}

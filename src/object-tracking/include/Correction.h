#ifndef CORRECTION_H
#define CORRECTION_H

#include <iCubPointCloud.h>
#include <BayesFilters/AdditiveMeasurementModel.h>
#include <BayesFilters/SUKFCorrection.h>


class MeasurementModelReference
{
public:
    MeasurementModelReference(bfl::AdditiveMeasurementModel& measurement_model);

    iCubPointCloud& getPointCloudModel();
private:
    iCubPointCloud& meas_model_;
};


class Correction : public MeasurementModelReference,
                   public bfl::SUKFCorrection
{
public:
    Correction
    (
        std::unique_ptr<bfl::AdditiveMeasurementModel> meas_model,
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
     ) noexcept;

    void correctStep(const bfl::GaussianMixture& pred_state, bfl::GaussianMixture& corr_state) override;

    Eigen::MatrixXd getNoiseCovarianceMatrix(const std::size_t index) override;

    void reset();
};

#endif /* CORRECTION_H */

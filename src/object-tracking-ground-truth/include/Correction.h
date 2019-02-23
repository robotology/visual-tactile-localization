#ifndef CORRECTION_H
#define CORRECTION_H

#include <BayesFilters/LinearMeasurementModel.h>
#include <BayesFilters/KFCorrection.h>

#include <Eigen/Dense>


class Correction : public bfl::KFCorrection
{
public:
    Correction(std::unique_ptr<bfl::LinearMeasurementModel> meas_model) noexcept;

    virtual ~Correction();

    void correctStep(const bfl::GaussianMixture& pred_state, bfl::GaussianMixture& corr_state) override;
};

#endif /* CORRECTION_H */

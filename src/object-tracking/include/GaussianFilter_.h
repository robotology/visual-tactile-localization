#ifndef GAUSSIANFILTER_H
#define GAUSSIANFILTER_H

#include <BayesFilters/FilteringAlgorithm.h>
#include <BayesFilters/Gaussian.h>
#include <BayesFilters/GaussianPrediction.h>
#include <Correction.h>

namespace bfl {
    class GaussianFilter_;
}


class bfl::GaussianFilter_: public bfl::FilteringAlgorithm
{
public:
    GaussianFilter_(Gaussian& initial_state, std::unique_ptr<GaussianPrediction> prediction, std::unique_ptr<Correction> correction) noexcept;

    GaussianFilter_(GaussianFilter_&& kf) noexcept;

    GaussianFilter_& operator=(GaussianFilter_&& gf) noexcept;

    virtual ~GaussianFilter_() noexcept;

    bool initialization() override;

    void filteringStep() override;

    bool runCondition() override;

    bool skip(const std::string& what_step, const bool status) override;

protected:
    Gaussian predicted_state_;

    Gaussian corrected_state_;

    std::unique_ptr<GaussianPrediction> prediction_;

    std::unique_ptr<Correction> correction_;
};

#endif /* GAUSSIANFILTER__H */

#ifndef SIMULATEDFILTER_H
#define SIMULATEDFILTER_H

#include <Correction.h>

#include <BayesFilters/Gaussian.h>
#include <BayesFilters/GaussianCorrection.h>
#include <GaussianFilter_.h>
#include <BayesFilters/GaussianPrediction.h>

#include <memory>

class SimulatedFilter : public bfl::GaussianFilter_
{
public:
    SimulatedFilter
    (
        bfl::Gaussian& initial_state,
        std::unique_ptr<bfl::GaussianPrediction> prediction,
        std::unique_ptr<Correction> correction,
        unsigned int simulation_steps
    );

    virtual ~SimulatedFilter();

protected:
    bool runCondition() override;

    std::vector<std::string> log_file_names(const std::string& prefix_path, const std::string& prefix_name) override;

    void filteringStep() override;

    void log() override;

private:
    unsigned int simulation_steps_;
};

#endif /* SIMULATEDFILTER_H */

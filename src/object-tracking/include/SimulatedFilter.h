#ifndef SIMULATEDFILTER_H
#define SIMULATEDFILTER_H

#include <BayesFilters/Gaussian.h>
#include <BayesFilters/GaussianCorrection.h>
#include <BayesFilters/GaussianFilter.h>
#include <BayesFilters/GaussianPrediction.h>

#include <memory>

class SimulatedFilter : public bfl::GaussianFilter
{
public:
    SimulatedFilter
    (
        bfl::Gaussian& initial_state,
        std::unique_ptr<bfl::GaussianPrediction> prediction,
        std::unique_ptr<bfl::GaussianCorrection> correction,
        unsigned int simulation_steps
    );

    virtual ~SimulatedFilter();

protected:
    bool runCondition() override;

    std::vector<std::string> log_filenames(const std::string& prefix_path, const std::string& prefix_name) override;

    void filteringStep() override;

    void log() override;

private:
    unsigned int simulation_steps_;
};

#endif /* SIMULATEDFILTER_H */

/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef SIMULATEDFILTER_H
#define SIMULATEDFILTER_H

#include <Correction.h>

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

    std::vector<std::string> log_file_names(const std::string& prefix_path, const std::string& prefix_name) override;

    void filteringStep() override;

    void log() override;

private:
    bfl::Gaussian predicted_state_;

    bfl::Gaussian corrected_state_;

    unsigned int simulation_steps_;
};

#endif /* SIMULATEDFILTER_H */

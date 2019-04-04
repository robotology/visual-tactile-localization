/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <SimulatedFilter.h>

#include <Eigen/Dense>

using namespace bfl;
using namespace Eigen;


SimulatedFilter::SimulatedFilter
(
    Gaussian& initial_state,
    std::unique_ptr<GaussianPrediction> prediction,
    std::unique_ptr<Correction> correction,
    unsigned int simulation_steps
) :
    GaussianFilter_(initial_state, std::move(prediction), std::move(correction)),
    simulation_steps_(simulation_steps)
{ }


SimulatedFilter::~SimulatedFilter()
{ }


bool SimulatedFilter::runCondition()
{
    if (getFilteringStep() < simulation_steps_)
        return true;
    else
        return false;
}


std::vector<std::string> SimulatedFilter::log_file_names(const std::string& prefix_path, const std::string& prefix_name)
{
    return  {prefix_path + "/" + prefix_name + "_pred_mean",
             prefix_path + "/" + prefix_name + "_cor_mean"};
}


void SimulatedFilter::filteringStep()
{
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

    GaussianFilter_::filteringStep();

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    std::cout << "Executed step in "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
              << " ms"
              << std::endl;

    // Allow the state model to evaluate the sampling time online
    prediction_->getStateModel().setProperty("tick");
}


void SimulatedFilter::log()
{
    logger(predicted_state_.mean().transpose(), corrected_state_.mean().transpose());
}

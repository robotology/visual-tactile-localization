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


std::vector<std::string> SimulatedFilter::log_filenames(const std::string& prefix_path, const std::string& prefix_name)
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
}


void SimulatedFilter::log()
{
    VectorXd predicted_mean(6);
    predicted_mean.segment(0, 3) = predicted_state_.mean().segment(0, 3);
    predicted_mean.segment(3, 3) = predicted_state_.mean().segment(9, 3);

    VectorXd corrected_mean(6);
    corrected_mean.segment(0, 3) = corrected_state_.mean().segment(0, 3);
    corrected_mean.segment(3, 3) = corrected_state_.mean().segment(9, 3);

    logger(predicted_mean.transpose(), corrected_mean.transpose());
}

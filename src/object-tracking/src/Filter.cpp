#include <Filter.h>

#include <Eigen/Dense>

#include <yarp/eigen/Eigen.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>

using namespace bfl;
using namespace Eigen;
using namespace yarp::eigen;
using namespace yarp::sig;

Filter::Filter
(
    const std::string port_prefix,
    Gaussian& initial_state,
    std::unique_ptr<GaussianPrediction> prediction,
    std::unique_ptr<GaussianCorrection> correction
) :
    GaussianFilter(initial_state, std::move(prediction), std::move(correction)),
    initial_state_(initial_state)
{
    // Open estimate output port
    if (!port_estimate_out_.open("/" + port_prefix + "/estimate:o"))
    {
        std::string err = "FILTER::CTOR::ERROR\n\tError: cannot estimate output port.";
        throw(std::runtime_error(err));
    }

    // Open RPC input port for commands
    if (!port_rpc_command_.open("/" + port_prefix + "/cmd:i"))
    {
        std::string err = "FILTER::CTOR::ERROR\n\tError: cannot open RPC command port.";
        throw(std::runtime_error(err));
    }

    if (!(this->yarp().attachAsServer(port_rpc_command_)));
    {
        std::string err = "FILTER::CTOR::ERROR\n\tError: cannot attach the RPC command port.";
        throw(std::runtime_error(err));
    }

    yInfo() << log_ID_ << "RPC command port opened and attached. Ready to recieve commands!";
}


Filter::~Filter()
{
    port_estimate_out_.close();
}


bool Filter::initialization()
{
    // Note that initialize() is also called when the a bfl::FilteringAlgorithm is reset
    corrected_state_ = initial_state_;
    predicted_state_ = Gaussian(initial_state_.dim_linear, initial_state_.dim_circular);
}


bool Filter::run_filter()
{
    run();

    return true;
}


bool Filter::reset_filter()
{
    reset();

    return true;
}


bool Filter::stop_filter()
{
    reboot();

    return true;
}


bool Filter::skip_step(const std::string& what_step, const bool status)
{
    return skip(what_step, status);
}


bool Filter::quit()
{
    return teardown();
}


std::vector<std::string> Filter::log_filenames(const std::string& prefix_path, const std::string& prefix_name)
{
    return  {prefix_path + "/" + prefix_name + "_pred_mean",
             prefix_path + "/" + prefix_name + "_cor_mean"};
}


void Filter::filteringStep()
{
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

    GaussianFilter::filteringStep();

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    std::cout << "Executed step in "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
              << " ms"
              << std::endl;

    // Send estimate over the port using axis/angle representation
    ;
    VectorXd estimate(7);
    estimate.head<3>() = corrected_state_.mean().head<3>();
    AngleAxisd angle_axis(AngleAxisd(corrected_state_.mean(3), Vector3d::UnitZ()) *
                          AngleAxisd(corrected_state_.mean(4), Vector3d::UnitY()) *
                          AngleAxisd(corrected_state_.mean(5), Vector3d::UnitX()));
    estimate.segment<3>(3) = angle_axis.axis();
    estimate(6) = angle_axis.angle();

    Vector& estimate_yarp = port_estimate_out_.prepare();
    estimate_yarp.resize(7);
    toEigen(estimate_yarp) = estimate;
    port_estimate_out_.write();
}


void Filter::log()
{
    VectorXd predicted_mean(6);
    predicted_mean.segment(0, 3) = predicted_state_.mean().segment(0, 3);
    predicted_mean.segment(3, 3) = predicted_state_.mean().segment(9, 3);

    VectorXd corrected_mean(6);
    corrected_mean.segment(0, 3) = corrected_state_.mean().segment(0, 3);
    corrected_mean.segment(3, 3) = corrected_state_.mean().segment(9, 3);

    logger(predicted_mean.transpose(), corrected_mean.transpose());
}

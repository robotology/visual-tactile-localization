#include <ArucoTracker.h>

#include <Eigen/Dense>

#include <chrono>

#include <yarp/eigen/Eigen.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>

using namespace bfl;
using namespace Eigen;
using namespace yarp::eigen;
using namespace yarp::os;
using namespace yarp::sig;


ArucoTracker::ArucoTracker
(
    const std::string port_prefix,
    Gaussian& initial_state,
    std::unique_ptr<GaussianPrediction> prediction,
    std::unique_ptr<Correction> correction
) :
    GaussianFilter_(initial_state, std::move(prediction), std::move(correction)),
    initial_state_(initial_state),
    pause_(false)
{
    // Open estimate output port
    if (!port_estimate_out_.open("/" + port_prefix + "/estimate:o"))
    {
        std::string err = log_ID_ + "::CTOR::ERROR\n\tError: cannot estimate output port.";
        throw(std::runtime_error(err));
    }

    // Open RPC input port for commands
    if (!port_rpc_command_.open("/" + port_prefix + "/cmd:i"))
    {
        std::string err = log_ID_ + "::CTOR::ERROR\n\tError: cannot open RPC command port.";
        throw(std::runtime_error(err));
    }

    if (!(this->yarp().attachAsServer(port_rpc_command_)))
    {
        std::string err = log_ID_ + "::CTOR::ERROR\n\tError: cannot attach the RPC command port.";
        throw(std::runtime_error(err));
    }

    yInfo() << log_ID_ << "RPC command port opened and attached. Ready to receive commands!";
}


ArucoTracker::~ArucoTracker()
{
    port_estimate_out_.close();
}


bool ArucoTracker::initialization()
{
    // Note that initialize() is also called when the a bfl::FilteringAlgorithm is reset
    corrected_state_ = initial_state_;
    predicted_state_ = Gaussian(initial_state_.dim_linear, initial_state_.dim_circular);

    // Reset the sample time of the prediction
    prediction_->getStateModel().setProperty("reset");


    return true;
}


bool ArucoTracker::run_filter()
{
    run();

    return true;
}


bool ArucoTracker::reset_filter()
{
    // Reset the sample time of the prediction
    prediction_->getStateModel().setProperty("reset");

    reset();

    return true;
}


bool ArucoTracker::stop_filter()
{
    // Reset the sample time of the prediction
    prediction_->getStateModel().setProperty("reset");

    reboot();

    return true;
}


void ArucoTracker::pause_filter()
{
    pause_ = true;
}


void ArucoTracker::resume_filter()
{
    pause_ = false;
}


bool ArucoTracker::skip_step(const std::string& what_step, const bool status)
{
    return skip(what_step, status);
}


std::vector<std::string> ArucoTracker::get_point_estimate_info()
{
    return {"This filter uses the conditional expected value as point estimate."};
}


bool ArucoTracker::set_point_estimate_method(const std::string& method)
{
    return false;
}


bool ArucoTracker::set_history_window(const int16_t window)
{
    return false;
}


bool ArucoTracker::quit()
{
    return teardown();
}


std::vector<std::string> ArucoTracker::log_filenames(const std::string& prefix_path, const std::string& prefix_name)
{
    return  {prefix_path + "/" + prefix_name + "_pred_mean",
             prefix_path + "/" + prefix_name + "_cor_mean"};
}


void ArucoTracker::filteringStep()
{
    if (pause_)
    {
        // do nothing
        std::this_thread::sleep_for(std::chrono::seconds(1));

        return;
    }
    // std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

    GaussianFilter_::filteringStep();

    // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    // std::cout << "Executed step in "
    //           << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
    //           << " ms"
    //           << std::endl;

    // Extract estimate
    VectorXd corrected_mean = corrected_state_.mean();

    // Send estimate over the port using axis/angle representation
    VectorXd estimate(7);
    estimate.head<3>() = corrected_mean.head<3>();
    AngleAxisd angle_axis(AngleAxisd(corrected_mean(9), Vector3d::UnitZ()) *
                          AngleAxisd(corrected_mean(10), Vector3d::UnitY()) *
                          AngleAxisd(corrected_mean(11), Vector3d::UnitX()));
    estimate.segment<3>(3) = angle_axis.axis();
    estimate(6) = angle_axis.angle();

    Vector& estimate_yarp = port_estimate_out_.prepare();
    estimate_yarp.resize(7);
    toEigen(estimate_yarp) = estimate;
    port_estimate_out_.write();

    // Allow the state model to evaluate the sampling time online
    prediction_->getStateModel().setProperty("tick");
}


void ArucoTracker::log()
{
    logger(predicted_state_.mean().transpose(), corrected_state_.mean().transpose());
}

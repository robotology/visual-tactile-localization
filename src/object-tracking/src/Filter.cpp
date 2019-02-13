#include <Filter.h>

#include <Eigen/Dense>

#include <chrono>

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
    std::unique_ptr<Correction> correction,
    std::unique_ptr<BoundingBoxEstimator> bbox_estimator,
    std::shared_ptr<iCubPointCloudExogenousData> icub_point_cloud_share
) :
    GaussianFilter_(initial_state, std::move(prediction), std::move(correction)),
    bbox_estimator_(std::move(bbox_estimator)),
    initial_state_(initial_state),
    icub_point_cloud_share_(icub_point_cloud_share),
    pause_(false)
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

    if (!(this->yarp().attachAsServer(port_rpc_command_)))
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

    // Reset the sample time of the prediction
    prediction_->getStateModel().setProperty("reset_time");

    return true;
}


bool Filter::run_filter()
{
    run();

    return true;
}


bool Filter::reset_filter()
{
    // Reet the bounding box estimator
    bbox_estimator_->reset();

    // Reset the correction step
    correction_->reset();

    // Reset the sample time of the prediction
    prediction_->getStateModel().setProperty("reset_time");

    reset();

    return true;
}


bool Filter::stop_filter()
{
    // Reset the sample time of the prediction
    prediction_->getStateModel().setProperty("reset_time");

    reboot();

    return true;
}


void Filter::pause_filter()
{
    pause_ = true;
}


void Filter::resume_filter()
{
    pause_ = false;
}


void Filter::contacts(const bool enable)
{
    icub_point_cloud_share_->setUseContacts(enable);
}


bool Filter::skip_step(const std::string& what_step, const bool status)
{
    return skip(what_step, status);
}


std::vector<std::string> Filter::get_point_estimate_info()
{
    return {"This filter uses the conditional expected value as point estimate."};
}


bool Filter::set_point_estimate_method(const std::string& method)
{
    return false;
}


bool Filter::set_history_window(const int16_t window)
{
    return false;
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
    if (pause_)
    {
        // do nothing
        std::this_thread::sleep_for(std::chrono::seconds(1));

        return;
    }
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

    bbox_estimator_->step();
    icub_point_cloud_share_->setBoundingBox(bbox_estimator_->getEstimate());

    GaussianFilter_::filteringStep();

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    std::cout << "Executed step in "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
              << " ms"
              << std::endl;

    // Extract estimate
    VectorXd corrected_mean = corrected_state_.mean();

    // Use estimate as hint for the bounding box estimator
    bbox_estimator_->setObjectPose(corrected_mean);
    bbox_estimator_->enableObjectFeedforward(!(icub_point_cloud_share_->getOcclusion()));
    bbox_estimator_->enableHandFeedforward(icub_point_cloud_share_->getContactState());

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


void Filter::log()
{
    logger(predicted_state_.mean().transpose(), corrected_state_.mean().transpose());
}

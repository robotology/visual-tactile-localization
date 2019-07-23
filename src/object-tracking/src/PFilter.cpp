/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifdef _OPENMP
#include <omp.h>
#endif

#include <PFilter.h>

#include <BayesFilters/utils.h>

#include <yarp/eigen/Eigen.h>
#include <yarp/os/LogStream.h>

using namespace bfl;
using namespace Eigen;
using namespace yarp::eigen;
using namespace yarp::sig;


PFilter::PFilter
(
    const std::string port_prefix,
    const std::size_t num_particle,
    const double resampling_threshold,
    const std::string point_estimate_method,
    const std::size_t point_estimate_window_size,
    std::unique_ptr<ParticleSetInitialization> initialization,
    std::unique_ptr<PFPrediction> prediction,
    std::unique_ptr<PFCorrection> correction,
    std::unique_ptr<Resampling> resampling,
    std::shared_ptr<PointCloudSegmentation> segmentation,
    std::unique_ptr<Validator2D> validator,
    std::unique_ptr<RateStabilizer> rate_stabilizer,
    const bool& enable_log,
    const std::string& log_path
) :
    SIS
    (
        num_particle,
        9, /* linear part of the state */
        3, /* circular part of the state */
        std::move(initialization),
        std::move(prediction),
        std::move(correction),
        std::move(resampling)
    ),
    validator_(std::move(validator)),
    rate_stabilizer_(std::move(rate_stabilizer)),
    resampling_threshold_(resampling_threshold),
    segmentation_(segmentation),
    point_estimate_extraction_(9, 3),
    rate_(1),
    pause_(false),
    enable_log_(enable_log),
    log_path_(log_path)
{
    // Setup point estimates extraction
    set_point_estimate_method(point_estimate_method);
    point_estimate_extraction_.setMobileAverageWindowSize(point_estimate_window_size);

    // Open estimate output port
    if (!port_estimate_out_.open("/" + port_prefix + "/estimate:o"))
    {
        std::string err = "PFILTER::CTOR::ERROR\n\tError: cannot estimate output port.";
        throw(std::runtime_error(err));
    }

    // Open timings output port
    if (!port_timings_out_.open("/" + port_prefix + "/timings:o"))
    {
        std::string err = "PFILTER::CTOR::ERROR\n\tError: cannot timings output port.";
        throw(std::runtime_error(err));
    }

    // Open RPC input port for commands
    if (!port_rpc_command_.open("/" + port_prefix + "/cmd:i"))
    {
        std::string err = "PFILTER::CTOR::ERROR\n\tError: cannot open RPC command port.";
        throw(std::runtime_error(err));
    }

    if (!(this->yarp().attachAsServer(port_rpc_command_)))
    {
        std::string err = "PFILTER::CTOR::ERROR\n\tError: cannot attach the RPC command port.";
        throw(std::runtime_error(err));
    }

    rate_.setMethod(EstimatesExtraction::ExtractionMethod::emean);
    rate_.setMobileAverageWindowSize(15);

    yInfo() << log_ID_ << "RPC command port opened and attached. Ready to recieve commands!";
}


PFilter::~PFilter()
{
    port_estimate_out_.close();
    port_timings_out_.close();
}


bool PFilter::run_filter()
{
    if (enable_log_)
        enable_log(log_path_, "object-tracking");
    correction_->getMeasurementModel().setProperty("enable_log");

    run();

    return true;
}


bool PFilter::reset_filter()
{
    SIS::reset();

    // Reset the kinematic model
    prediction_->getStateModel().setProperty("reset");

    // Reset the measurement model
    correction_->getMeasurementModel().setProperty("reset");

    // Reset the point estimate
    point_estimate_extraction_.clear();

    // Reset the rate stabilizer
    if (rate_stabilizer_ != nullptr)
        rate_stabilizer_ ->reset();
    rate_.clear();

    // Reset the segmentation
    segmentation_->reset();

    // Reset the validator
    validator_->reset();

    keyframe_counter_ = 1;

    disable_log();

    return true;
}


bool PFilter::stop_filter()
{
    reboot();

    // Reset the kinematic model
    prediction_->getStateModel().setProperty("reset");

    // Reset the measurement model
    correction_->getMeasurementModel().setProperty("reset");

    // Reset the point estimate
    point_estimate_extraction_.clear();

    // Reset the rate stabilizer
    if (rate_stabilizer_ != nullptr)
        rate_stabilizer_->reset();
    rate_.clear();

    // Reset the segmentation
    segmentation_->reset();

    // Reset the validator
    validator_->reset();

    keyframe_counter_ = 1;

    disable_log();

    return true;
}


void PFilter::pause_filter()
{
    pause_ = true;
}


void PFilter::resume_filter()
{
    pause_ = false;
}


void PFilter::contacts(const bool enable)
{
    if (enable)
        correction_->getMeasurementModel().setProperty("use_contacts_on");
    else
        correction_->getMeasurementModel().setProperty("use_contacts_off");
}


bool PFilter::skip_step(const std::string& what_step, const bool status)
{
    return skip(what_step, status);
}


std::vector<std::string> PFilter::get_point_estimate_info()
{
    return point_estimate_extraction_.getInfo();
}


bool PFilter::set_point_estimate_method(const std::string& method)
{
    if (method == "mean")
    {
        point_estimate_extraction_.setMethod(EstimatesExtraction::ExtractionMethod::mean);

        return true;
    }
    else if (method == "smean")
    {
        point_estimate_extraction_.setMethod(EstimatesExtraction::ExtractionMethod::smean);

        return true;
    }
    else if (method == "wmean")
    {
        point_estimate_extraction_.setMethod(EstimatesExtraction::ExtractionMethod::wmean);

        return true;
    }
    else if (method == "emean")
    {
        point_estimate_extraction_.setMethod(EstimatesExtraction::ExtractionMethod::emean);

        return true;
    }
    else if (method == "mode")
    {
        point_estimate_extraction_.setMethod(EstimatesExtraction::ExtractionMethod::mode);

        return true;
    }
    else if (method == "smode")
    {
        point_estimate_extraction_.setMethod(EstimatesExtraction::ExtractionMethod::smode);

        return true;
    }
    else if (method == "wmode")
    {
        point_estimate_extraction_.setMethod(EstimatesExtraction::ExtractionMethod::wmode);

        return true;
    }
    else if (method == "emode")
    {
        point_estimate_extraction_.setMethod(EstimatesExtraction::ExtractionMethod::emode);

        return true;
    }
    else if ((method == "map") || (method == "smap") || (method == "wmap") || (method == "emap"))
    {
        /* These extraction methods are not supported. */
        return false;
    }

    return false;
}


bool PFilter::set_history_window(const int16_t window)
{
    if (window > 0)
        return point_estimate_extraction_.setMobileAverageWindowSize(window);
    else
        return false;
}


bool PFilter::quit()
{
    disable_log();

    return teardown();
}


std::vector<std::string> PFilter::log_file_names(const std::string& prefix_path, const std::string& prefix_name)
{
    return  {prefix_path + "/" + prefix_name + "_estimate"};
}


void PFilter::filteringStep()
{
    if (pause_)
    {
        // do nothing
        std::this_thread::sleep_for(std::chrono::seconds(1));

        return;
    }

    // Stop when no measurements are available
    // Unfortunately no getProperty available
    if(!correction_->getMeasurementModel().setProperty("measurements_available"))
    {
        quit();

        return;
    }

#ifdef _OPENMP
    double start = omp_get_wtime();
#else
    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
#endif

    if (getFilteringStep() != 0)
        prediction_->predict(cor_particle_, pred_particle_);

    correction_->correct(pred_particle_, cor_particle_);

    /* Normalize weights using LogSumExp. */
    cor_particle_.weight().array() -= utils::log_sum_exp(cor_particle_.weight());

    double neff = resampling_->neff(cor_particle_.weight());
    if (neff < static_cast<double>(num_particle_) * resampling_threshold_)
    {
        ParticleSet res_particle(num_particle_, state_size_);
        VectorXi res_parent(num_particle_, 1);

        resampling_->resample(cor_particle_, res_particle, res_parent);

        cor_particle_ = res_particle;
    }

    // Use mean estimate as hint for the point cloud segmentation scheme
    Transform<double, 3, Affine> object_transform;
    int max_weight_index;
    cor_particle_.weight().maxCoeff(&max_weight_index);
    const VectorXd& mean = cor_particle_.mean(max_weight_index);
    object_transform = Translation<double, 3>(mean.head<3>());
    AngleAxisd rotation(AngleAxisd(mean(9), Vector3d::UnitZ()) *
                        AngleAxisd(mean(10), Vector3d::UnitY()) *
                        AngleAxisd(mean(11), Vector3d::UnitX()));
    object_transform.rotate(rotation);
    segmentation_->setObjectPose(object_transform);

    // Update the point estimate extraction
    bool valid_estimate;
    std::tie(valid_estimate, point_estimate_) =  point_estimate_extraction_.extract(cor_particle_.state(), cor_particle_.weight());
    if (!valid_estimate)
        yInfo() << log_ID_ << "Cannot extract point estimate!";

    // Render 2d evaluation using current estimate
    object_transform = Translation<double, 3>(point_estimate_.head<3>());
    rotation = AngleAxisd(AngleAxisd(point_estimate_(9), Vector3d::UnitZ()) *
                          AngleAxisd(point_estimate_(10), Vector3d::UnitY()) *
                          AngleAxisd(point_estimate_(11), Vector3d::UnitX()));
    object_transform.rotate(rotation);
    validator_->renderEvaluation(object_transform);

    if (valid_estimate)
    {
        // Send estimate over the port using axis/angle representation
        VectorXd estimate(14);
        estimate.head<3>() = point_estimate_.head<3>();
        AngleAxisd angle_axis(AngleAxisd(point_estimate_(9), Vector3d::UnitZ()) *
                              AngleAxisd(point_estimate_(10), Vector3d::UnitY()) *
                              AngleAxisd(point_estimate_(11), Vector3d::UnitX()));
        estimate.segment<3>(3) = angle_axis.axis();
        estimate(6) = angle_axis.angle();
        estimate.segment<3>(7) = point_estimate_.segment<3>(3);
        estimate.segment<3>(10) = point_estimate_.segment<3>(6);
        estimate(13) = keyframe_counter_;
        std::cout << "Frame: " << keyframe_counter_ << std::endl;

        // Log
        logger(estimate.transpose());

        Vector& estimate_yarp = port_estimate_out_.prepare();
        estimate_yarp.resize(14);
        toEigen(estimate_yarp) = estimate;
        port_estimate_out_.write();
    }
    else
        std::cout << "Unable to extract the estimate!" << std::endl;

    if ((segmentation_->getProperty("is_occlusion")) && !(correction_->getMeasurementModel().setProperty("get_contact_state")))
        prediction_->getStateModel().setProperty("tdd_advance");
    else
        prediction_->getStateModel().setProperty("tdd_reset");

    prediction_->getStateModel().setProperty("tick");

    // Evaluate execution time
#ifdef _OPENMP
    double execution_time = omp_get_wtime() - start;
#else
    std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
    double execution_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000.0;
#endif
    VectorXd rate(1);
    VectorXd rate_vector(1);
    VectorXd rate_weight(1);
    rate_vector(0) = 1 / execution_time;
    rate_weight(0) = std::log(1.0);
    std::tie(std::ignore, rate) = rate_.extract(rate_vector, rate_weight);

    std::cout << "Current rate is " << rate(0) << " fps" << std::endl;
    std::cout << "Neff is: " << neff << std::endl << std::endl;

    // Change depth stride
    if (rate_stabilizer_ != nullptr)
    {
        int depth_stride = segmentation_->getDepthStride();
        int control = rate_stabilizer_->getOutput(rate(0));
        depth_stride += control;

        if (depth_stride <= 0)
            depth_stride = 1;

        std::cout << "Depth stride is " << depth_stride << std::endl;
        segmentation_->setDepthStride(depth_stride);
    }

    // Send execution time
    Vector& timings = port_timings_out_.prepare();
    timings.resize(1);
    timings[0] = execution_time;
    port_timings_out_.write();

    /* Increase keyframe counter */
    keyframe_counter_++;
}

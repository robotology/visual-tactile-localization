/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

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
    std::unique_ptr<Validator2D> validator
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
    resampling_threshold_(resampling_threshold),
    segmentation_(segmentation),
    point_estimate_extraction_(9, 3),
    pause_(false)
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

    yInfo() << log_ID_ << "RPC command port opened and attached. Ready to recieve commands!";
}


PFilter::~PFilter()
{
    port_estimate_out_.close();
    port_timings_out_.close();
}


bool PFilter::run_filter()
{
    run();

    return true;
}


bool PFilter::reset_filter()
{
    // Reset the kinematic model
    prediction_->getStateModel().setProperty("reset");

    // Reset the measurement model
    correction_->getMeasurementModel().setProperty("reset");

    // Reset the point estimate
    point_estimate_extraction_.clear();

    SIS::reset();

    return true;
}


bool PFilter::stop_filter()
{
    // Reset the kinematic model
    prediction_->getStateModel().setProperty("reset");

    // Reset the measurement model
    correction_->getMeasurementModel().setProperty("reset");

    // Reset the point estimate
    point_estimate_extraction_.clear();

    reboot();

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
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

    if (getFilteringStep() != 0)
        prediction_->predict(cor_particle_, pred_particle_);

    correction_->correct(pred_particle_, cor_particle_);

    /* Normalize weights using LogSumExp. */
    cor_particle_.weight().array() -= utils::log_sum_exp(cor_particle_.weight());

    log();

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

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    std::cout << "Executed step in "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
              << " ms"
              << std::endl;
    std::cout << "Neff is: " << neff<< std::endl << std::endl;

    // Send execution time
    double execution_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    Vector& timings = port_timings_out_.prepare();
    timings.resize(1);
    timings[0] = execution_time / 1000.0;
    port_timings_out_.write();

    if (valid_estimate)
    {
        // Log
        logger(point_estimate_.transpose());

        // Send estimate over the port using axis/angle representation
        VectorXd estimate(13);
        estimate.head<3>() = point_estimate_.head<3>();
        AngleAxisd angle_axis(AngleAxisd(point_estimate_(9), Vector3d::UnitZ()) *
                              AngleAxisd(point_estimate_(10), Vector3d::UnitY()) *
                              AngleAxisd(point_estimate_(11), Vector3d::UnitX()));
        estimate.segment<3>(3) = angle_axis.axis();
        estimate(6) = angle_axis.angle();
        estimate.segment<3>(7) = point_estimate_.segment<3>(3);
        estimate.segment<3>(10) = point_estimate_.segment<3>(6);

        Vector& estimate_yarp = port_estimate_out_.prepare();
        estimate_yarp.resize(13);
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
}


void PFilter::log()
{ }

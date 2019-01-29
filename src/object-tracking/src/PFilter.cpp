#include <PFilter.h>

#include <yarp/eigen/Eigen.h>

using namespace bfl;
using namespace Eigen;
using namespace yarp::eigen;
using namespace yarp::sig;


PFilter::PFilter
(
    const std::string port_prefix,
    const std::size_t num_particle,
    std::unique_ptr<ParticleSetInitialization> initialization,
    std::unique_ptr<PFPrediction> prediction,
    std::unique_ptr<ParticlesCorrection> correction,
    std::unique_ptr<Resampling> resampling,
    std::shared_ptr<iCubPointCloudExogenousData> icub_point_cloud_share
) :
    ParticleCorrectionReset(correction.get()),
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
    icub_point_cloud_share_(icub_point_cloud_share),
    point_estimate_extraction_(9, 3)
{
    // Setup point estimates extraction
    point_estimate_extraction_.setMethod(EstimatesExtraction::ExtractionMethod::mode);

    // Open estimate output port
    if (!port_estimate_out_.open("/" + port_prefix + "/estimate:o"))
    {
        std::string err = "PFILTER::CTOR::ERROR\n\tError: cannot estimate output port.";
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
}


bool PFilter::run_filter()
{
    run();

    return true;
}


bool PFilter::reset_filter()
{
    // Reset the correction step
    reset_correction();

    // Reset the point estimate
    point_estimate_extraction_.clear();

    SIS::reset();

    return true;
}


bool PFilter::stop_filter()
{
    reboot();

    return true;
}


bool PFilter::skip_step(const std::string& what_step, const bool status)
{
    return skip(what_step, status);
}


bool PFilter::quit()
{
    return teardown();
}


std::vector<std::string> PFilter::log_filenames(const std::string& prefix_path, const std::string& prefix_name)
{
    return  {prefix_path + "/" + prefix_name + "_estimate"};
}


void PFilter::filteringStep()
{
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

    SIS::filteringStep();

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    std::cout << "Executed step in "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
              << " ms"
              << std::endl;

    // Update the point estimate extraction
    bool valid_estimate;
    VectorXd point_estimate;
    std::tie(valid_estimate, point_estimate) =  point_estimate_extraction_h.extract(cor_particle_.state(), cor_particle_.weight());

    // Log
    logger(point_estimate_.transpose());

    // Update shared data with iCubPointCloudData
    icub_point_cloud_share_->setObjectEstimate(point_estimate);

    // Send estimate over the port using axis/angle representation
    VectorXd estimate(7);
    estimate.head<3>() = point_estimate.head<3>();
    AngleAxisd angle_axis(AngleAxisd(point_estimate(9), Vector3d::UnitZ()) *
                          AngleAxisd(point_estimate(10), Vector3d::UnitY()) *
                          AngleAxisd(point_estimate(11), Vector3d::UnitX()));
    estimate.segment<3>(3) = angle_axis.axis();
    estimate(6) = angle_axis.angle();

    Vector& estimate_yarp = port_estimate_out_.prepare();
    estimate_yarp.resize(7);
    toEigen(estimate_yarp) = estimate;
    port_estimate_out_.write();
}


void PFilter::log()
{ }

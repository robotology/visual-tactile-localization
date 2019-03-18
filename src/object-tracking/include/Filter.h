#ifndef FILTER_H
#define FILTER_H

#include <BoundingBoxEstimator.h>
#include <Correction.h>
#include <GaussianFilter_.h>

#include <BayesFilters/Gaussian.h>
// #include <BayesFilters/GaussianCorrection.h>
/* #include <BayesFilters/GaussianFilter.h> */
#include <BayesFilters/GaussianPrediction.h>
#include <iCubPointCloud.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/sig/Vector.h>

#include <thrift/ObjectTrackingIDL.h>

#include <memory>

class Filter : public bfl::GaussianFilter_,
               public ObjectTrackingIDL
{
public:
    Filter
    (
        const std::string port_prefix,
        bfl::Gaussian& initial_state,
        std::unique_ptr<bfl::GaussianPrediction> prediction,
        std::unique_ptr<Correction> correction,
        std::unique_ptr<BoundingBoxEstimator> bbox_estimator,
        std::shared_ptr<iCubPointCloudExogenousData> icub_point_cloud_share
    );

    virtual ~Filter();

    bool initialization() override;

    bool run_filter() override;

    bool reset_filter() override;

    bool stop_filter() override;

    void pause_filter() override;

    void resume_filter() override;

    void contacts(const bool enable) override;

    bool skip_step(const std::string& what_step, const bool status) override;

    std::vector<std::string> get_point_estimate_info() override;

    bool set_point_estimate_method(const std::string& method) override;

    bool set_history_window(const int16_t window);

    bool quit() override;

protected:
    std::vector<std::string> log_file_names(const std::string& prefix_path, const std::string& prefix_name) override;

    void filteringStep() override;

    void log() override;

    yarp::os::BufferedPort<yarp::sig::Vector> port_estimate_out_;

    yarp::os::Port port_rpc_command_;

    std::unique_ptr<BoundingBoxEstimator> bbox_estimator_;

    std::shared_ptr<iCubPointCloudExogenousData> icub_point_cloud_share_;

private:
    bfl::Gaussian initial_state_;

    const std::string log_ID_ = "[Filter]";

    bool pause_;
};

#endif /* FILTER_H */

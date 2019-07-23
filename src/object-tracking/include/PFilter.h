/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef PFILTER_H
#define PFILTER_H

#include <BayesFilters/EstimatesExtraction.h>
#include <BayesFilters/MeasurementModel.h>
#include <BayesFilters/ParticleSetInitialization.h>
#include <BayesFilters/PFPrediction.h>
#include <BayesFilters/PFCorrection.h>
#include <BayesFilters/Resampling.h>
#include <BayesFilters/SIS.h>

#include <PointCloudSegmentation.h>
#include <RateStabilizer.h>
#include <Validator2D.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/sig/Vector.h>

#include <thrift/ObjectTrackingIDL.h>


class PFilter : public bfl::SIS,
                public ObjectTrackingIDL
{
public:
    PFilter
    (
        const std::string port_prefix,
        const std::size_t num_particle,
        const double resampling_threshold,
        const std::string point_estimate_method,
        const std::size_t point_estimate_window_size,
        std::unique_ptr<bfl::ParticleSetInitialization> initialization,
        std::unique_ptr<bfl::PFPrediction> prediction,
        std::unique_ptr<bfl::PFCorrection> correction,
        std::unique_ptr<bfl::Resampling> resampling,
        std::shared_ptr<PointCloudSegmentation> segmentation,
        std::unique_ptr<Validator2D> validator,
        std::unique_ptr<RateStabilizer> rate_stabilizer,
        const bool& enable_log = false,
        const std::string& log_path = ""
    );

    virtual ~PFilter();

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

    yarp::os::BufferedPort<yarp::sig::Vector> port_estimate_out_;

    yarp::os::BufferedPort<yarp::sig::Vector> port_timings_out_;

    yarp::os::Port port_rpc_command_;

    std::shared_ptr<PointCloudSegmentation> segmentation_;

    std::unique_ptr<Validator2D> validator_;

    std::unique_ptr<RateStabilizer> rate_stabilizer_;

    bfl::EstimatesExtraction point_estimate_extraction_;

    bfl::EstimatesExtraction rate_;

    Eigen::VectorXd point_estimate_;

    double resampling_threshold_;

    bool pause_;

    bool enable_log_;

    const std::string log_path_;

    std::size_t keyframe_counter_ = 1;

private:
    const std::string log_ID_ = "[PFilter]";
};

#endif /* PFILTER_H */

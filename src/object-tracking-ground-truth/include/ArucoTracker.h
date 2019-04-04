/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ARUCOTRACKER_H
#define ARUCOTRACKER_H

#include <BayesFilters/Gaussian.h>
/* #include <BayesFilters/GaussianCorrection.h> */
/* #include <BayesFilters/GaussianFilter.h> */
#include <BayesFilters/GaussianPrediction.h>

#include <Correction.h>
#include <GaussianFilter_.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

#include <thrift/ArucoTrackerIDL.h>

#include <memory>


class ArucoTracker : public bfl::GaussianFilter_,
                     public ArucoTrackerIDL
{
public:
    ArucoTracker
    (
        const std::string port_prefix,
        bfl::Gaussian& initial_state,
        std::unique_ptr<bfl::GaussianPrediction> prediction,
        std::unique_ptr<Correction> correction
    );

    virtual ~ArucoTracker();

    bool initialization() override;

    bool run_filter() override;

    void filteringStep() override;

    bool reset_filter() override;

    bool stop_filter() override;

    void pause_filter() override;

    void resume_filter() override;

    bool skip_step(const std::string& what_step, const bool status) override;

    std::vector<std::string> get_point_estimate_info() override;

    bool set_point_estimate_method(const std::string& method) override;

    bool set_history_window(const int16_t window);

    bool quit() override;

protected:
    std::vector<std::string> log_file_names(const std::string& prefix_path, const std::string& prefix_name) override;

    void log() override;

    yarp::os::BufferedPort<yarp::sig::Vector> port_estimate_out_;

    yarp::os::Port port_rpc_command_;

private:
    bfl::Gaussian initial_state_;

    const std::string log_ID_ = "[ArucoTracker]";

    bool pause_;
};

#endif /* ARUCOFILTER_H */

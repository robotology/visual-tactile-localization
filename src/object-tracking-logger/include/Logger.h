/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef OBJECTTRACKINGLOGGER_H
#define OBJECTTRACKINGLOGGER_H

#include <BayesFilters/Logger.h>

#include <Eigen/Dense>

#include <thrift/ObjectTrackingLoggerIDL.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/Port.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>

#include <vector>
#include <string>


class ObjectTrackingLogger : public yarp::os::RFModule,
                             public ObjectTrackingLoggerIDL,
                             public bfl::Logger
{
public:
    ObjectTrackingLogger(const std::string port_prefix, const double period);

    virtual ~ObjectTrackingLogger();

    bool run() override;

    bool stop() override;

    bool quit() override;

    bool configure(yarp::os::ResourceFinder& rf) override;

    double getPeriod() override;

    bool updateModule() override;

    bool close() override;

protected:
    Eigen::Vector3d axisAngleToEuler210(const Eigen::VectorXd& axis_angle);

    std::vector<std::string> log_filenames(const std::string& prefix_path, const std::string& prefix_name) /* override */
    {
        return {prefix_path + "/" + prefix_name + "_estimate",
                prefix_path + "/" + prefix_name + "_gt_0",
                prefix_path + "/" + prefix_name + "_gt_1",
                prefix_path + "/" + prefix_name + "_execution"};
    }

    const std::string log_ID_ = "[LOGGER]";

    const std::string port_prefix_;

    double period_;

    yarp::os::Mutex mutex_;

    bool run_;

    bool quit_;

    yarp::os::BufferedPort<yarp::sig::Vector> port_estimate_in_;

    yarp::os::BufferedPort<yarp::sig::Vector> port_ground_truth_0_in_;

    yarp::os::BufferedPort<yarp::sig::Vector> port_ground_truth_1_in_;

    yarp::os::BufferedPort<yarp::sig::Vector> port_execution_time_in_;

    yarp::os::Port port_rpc_command_;
};

#endif

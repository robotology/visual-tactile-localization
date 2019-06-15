/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef RATESTABILIZER_H
#define RATESTABILIZER_H

#include <chrono>

#include <yarp/os/Port.h>

#include <thrift/RateStabilizerIDL.h>


class RateStabilizer : public RateStabilizerIDL
{
public:
    RateStabilizer(const std::string& port_prefix, const double& period, const double& set_point);

    virtual ~RateStabilizer();

    int getOutput(const double& feedback);

    /* Thrift interface. */

    void set_fps(const double fps) override;

    void reset() override;

protected:
    int doControl(const double& feedback);

private:
    /**
     * Desired period of the rate stabilizer in seconds.
     */
    double period_;

    /**
     * Desired frequency of the algorithm under control in Hz.
     */
    double set_point_;

    /**
     * Last measured system clock.
     */
    std::chrono::high_resolution_clock::time_point last_time_;

    /**
     * Elapsed time since last reset.
     */
    double elapsed_time_ = 0.0;

    yarp::os::Port port_rpc_command_;
};

#endif /* RATESTABILIZER_H */

/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <RateStabilizer.h>


RateStabilizer::RateStabilizer(const std::string& port_prefix, const double& period, const double& set_point) :
    period_(period),
    set_point_(set_point)
{
    // Open RPC input port for commands
    if (!port_rpc_command_.open("/" + port_prefix + "/rate-stabilizer/cmd:i"))
    {
        std::string err = "PFILTER::CTOR::ERROR\n\tError: cannot open RPC command port.";
        throw(std::runtime_error(err));
    }

    if (!(this->yarp().attachAsServer(port_rpc_command_)))
    {
        std::string err = "PFILTER::CTOR::ERROR\n\tError: cannot attach the RPC command port.";
        throw(std::runtime_error(err));
    }

}


RateStabilizer::~RateStabilizer()
{
    port_rpc_command_.close();
}


int RateStabilizer::getOutput(const double& feedback)
{
    int output = 0;

    std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
    double elapsed_time_partial = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time_).count() / 1000.0;

    elapsed_time_ += elapsed_time_partial;

    if (elapsed_time_ > period_)
    {
        output = doControl(feedback);

        elapsed_time_ = 0.0;
    }

    last_time_ = now;

    return output;
}


void RateStabilizer::set_fps(const double fps)
{
    set_point_ = fps;
}


void RateStabilizer::reset()
{
    elapsed_time_ = 0.0;
}


int RateStabilizer::doControl(const double& feedback)
{
    double error = set_point_ - feedback;
    int control = 0;

    if (error >= 0)
        control = 1;
    else
        control = -1;

    return control;
}

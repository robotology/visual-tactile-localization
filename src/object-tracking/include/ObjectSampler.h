/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef OBJECTSAMPLER_H
#define OBJECTSAMPLER_H

#include <Eigen/Dense>

class ObjectSampler
{
public:

    virtual ~ObjectSampler();
x
    virtual std::pair<bool, Eigen::MatrixXd> sample(const std::size_t& number_of_points) = 0;
};

#endif

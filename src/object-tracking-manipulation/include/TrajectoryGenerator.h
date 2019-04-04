/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef TRAJECTORYGENERATOR_H
#define TRAJECTORYGENERATOR_H

#include <Eigen/Dense>

class TrajectoryGenerator
{
public:
    virtual Eigen::VectorXd getCurrentPose(const double time) = 0;

    virtual void reset() = 0;
};

#endif

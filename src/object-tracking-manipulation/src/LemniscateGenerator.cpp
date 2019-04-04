/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <LemniscateGenerator.h>

using namespace Eigen;

LemniscateGenerator::LemniscateGenerator() :
    LemniscateGenerator(1.0, 1.0, false)
{ }


LemniscateGenerator::LemniscateGenerator(const double scale, const double time_scale, const bool invert_y_z_axes) :
    scale_(scale),
    time_scale_(time_scale),
    invert_y_z_axes_(invert_y_z_axes),
    center_(Vector3d::Zero())
{ }


void LemniscateGenerator::setCenter(const Eigen::Vector3d& center)
{
    center_ = center;
}


LemniscateGenerator::~LemniscateGenerator()
{ }


VectorXd LemniscateGenerator::getCurrentPose(const double time)
{
    //see https://gamedev.stackexchange.com/questions/43691/how-can-i-move-an-object-in-an-infinity-or-figure-8-trajectory
    double lemniscate_scale = 2 / ( 3 - cos(2 * time_scale_ * time));

    Vector3d pose;

    // stay on the x = center(0) plane
    pose(0) = center_(0);

    if (invert_y_z_axes_)
    {
        pose(1) = center_(1) + scale_ * lemniscate_scale * sin(2 * time_scale_ * time) / 2.0;
        pose(2) = center_(2) + scale_ * lemniscate_scale * cos(time_scale_ * time);
    }
    else
    {
        pose(1) = center_(1) + scale_ * lemniscate_scale * cos(time_scale_ * time);
        pose(2) = center_(2) + scale_ * lemniscate_scale * sin(2 * time_scale_ * time) / 2.0;
    }

    return pose;
}


void LemniscateGenerator::reset()
{ }

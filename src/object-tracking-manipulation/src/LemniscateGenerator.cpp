#include <LemniscateGenerator.h>

using namespace Eigen;

LemniscateGenerator::LemniscateGenerator() :
    LemniscateGenerator(1.0, 1.0)
{ }


LemniscateGenerator::LemniscateGenerator(const double scale, const double time_scale) :
    scale_(scale), time_scale_(time_scale),
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

    pose(1) = center_(1) + scale_ * lemniscate_scale * sin(2 * time_scale_ * time) / 2.0;
    pose(2) = center_(2) + scale_ * lemniscate_scale * cos(time_scale_ * time);

    return pose;
}


void LemniscateGenerator::reset()
{ }

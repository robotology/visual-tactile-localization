#include <SinusoidalTrajectory.h>

using namespace Eigen;

SinusoidalTrajectory::SinusoidalTrajectory(const VectorXd& scale, const VectorXd& time_scale) :
    scale_(scale), time_scale_(time_scale)
{ }


SinusoidalTrajectory::~SinusoidalTrajectory()
{ }


void SinusoidalTrajectory::setCenter(const VectorXd& center)
{
    center_ = center;
}


Eigen::VectorXd SinusoidalTrajectory::getCurrentPose(const double time)
{
    VectorXd pose(center_.size());

    for (std::size_t i = 0; i < center_.size(); i++)
        pose(i) = center_(i) + sin(time_scale_(i) * time) * scale_(i);

    return pose;
}


void SinusoidalTrajectory::reset()
{ }

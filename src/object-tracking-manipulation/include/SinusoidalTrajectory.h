#ifndef SINUSOIDALTRAJECTORY_H
#define SINUSOIDALTRAJECTORY_H

#include <Eigen/Dense>

#include <TrajectoryGenerator.h>

class SinusoidalTrajectory : public TrajectoryGenerator
{
public:
    SinusoidalTrajectory(const Eigen::VectorXd& scale, const Eigen::VectorXd& time_scale);

    Eigen::VectorXd getCurrentPose(const double time) override;

    void setCenter(const Eigen::VectorXd& center);

    void reset() override;

    virtual ~SinusoidalTrajectory();

protected:
    Eigen::VectorXd scale_;

    Eigen::VectorXd time_scale_;

    Eigen::VectorXd center_;
};

#endif /* SINUSOIDALTRAJECTORY_H */

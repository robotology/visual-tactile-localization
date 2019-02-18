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

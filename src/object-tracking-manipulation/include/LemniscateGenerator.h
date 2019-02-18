#ifndef LEMNISCATEGENERATOR_H
#define LEMNISCATEGENERATOR_H

#include <TrajectoryGenerator.h>

#include <Eigen/Dense>

class LemniscateGenerator : public TrajectoryGenerator
{
public:
    LemniscateGenerator();

    LemniscateGenerator(const double scale, const double time_scale);

    void setCenter(const Eigen::Vector3d& center);

    virtual ~LemniscateGenerator();

    Eigen::VectorXd getCurrentPose(const double time) override;

    void reset() override;

protected:
    double scale_;

    double time_scale_;

    Eigen::Vector3d center_;
};

#endif

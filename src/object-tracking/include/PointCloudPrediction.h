#ifndef POINTCLOUDPREDICTION_H
#define POINTCLOUDPREDICTION_H

#include <Eigen/Dense>

using ConstMatrixXdRef = const Eigen::Ref<const Eigen::MatrixXd>&;
using ConstVectorXdRef = const Eigen::Ref<const Eigen::VectorXd>&;

class PointCloudPrediction
{
public:
    virtual std::pair<bool, Eigen::MatrixXd> predictPointCloud(ConstMatrixXdRef state, ConstVectorXdRef meas) = 0;
};

#endif

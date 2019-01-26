#include <DiscreteKinematicModel.h>

#include <Eigen/Dense>

using namespace Eigen;


DiscreteKinematicModel::DiscreteKinematicModel
(
    const double T,
    const double q_x,
    const double q_y,
    const double q_z,
    const double q_x_dot,
    const double q_y_dot,
    const double q_z_dot,
    const double q_yaw,
    const double q_pitch,
    const double q_roll,
    const double q_yaw_dot,
    const double q_pitch_dot,
    const double q_roll_dot
)
{
    // Compute the state transition matrix
    F_ = MatrixXd::Identity(12, 12);

    // Compute the noise covariance matrix
    Q_ = MatrixXd::Zero(12, 12);

    Q_(0, 0) = q_x;
    Q_(1, 1) = q_y;
    Q_(2, 2) = q_z;
    Q_(9, 9) = q_yaw;
    Q_(10, 10) = q_pitch;
    Q_(11, 11) = q_roll;
}


DiscreteKinematicModel::~DiscreteKinematicModel()
{ }


std::pair<std::size_t, std::size_t> DiscreteKinematicModel::getOutputSize() const
{
    // 9 linear components (x, y, z, x_dot, y_dot, z_dot, yaw_dot, pitch_dot, roll_dot)
    // 3 angular components (yaw, pitch, roll)
    return std::make_pair(9, 3);
}


Eigen::MatrixXd DiscreteKinematicModel::getStateTransitionMatrix()
{
    return F_;
}


Eigen::MatrixXd DiscreteKinematicModel::getNoiseCovarianceMatrix()
{
    return Q_;
}


bool DiscreteKinematicModel::setProperty(const std::string& property)
{
    return false;
}

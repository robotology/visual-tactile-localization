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
    // F_ = MatrixXd::Identity(12, 12);
    F_ = MatrixXd::Zero(12, 12);
    F_.block<3, 3>(0, 0) = Matrix3d::Identity();
    F_.block<3, 3>(0, 3) = T * Matrix3d::Identity();
    F_.block<3, 3>(3, 3) = Matrix3d::Identity();
    F_.block<3, 3>(6, 6) = Matrix3d::Identity();
    F_.block<3, 3>(9, 6) = T * Matrix3d::Identity();
    F_.block<3, 3>(9, 9) = Matrix3d::Identity();

    // Compute the noise covariance matrix
    VectorXd Q_vector(12);
    Q_vector << q_x, q_y, q_z, q_x_dot, q_y_dot, q_z_dot, q_yaw_dot, q_pitch_dot, q_roll_dot, q_yaw, q_pitch, q_roll;
    Q_ = Q_vector.asDiagonal();
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

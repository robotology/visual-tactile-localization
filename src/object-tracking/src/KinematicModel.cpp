#include <KinematicModel.h>

#include <Eigen/Dense>

using namespace Eigen;


KinematicModel::KinematicModel
(
    const double T,
    const double sigma_x,  const double sigma_y,  const double sigma_z,
    const double sigma_yaw, const double sigma_pitch, const double sigma_roll
)
{
    // Compute the state transition matrix
    F_ = MatrixXd::Zero(12, 12);
    F_.block<3, 3>(0, 0) = Matrix3d::Identity();
    F_.block<3, 3>(0, 3) = T * Matrix3d::Identity();
    F_.block<3, 3>(3, 3) = Matrix3d::Identity();
    F_.block<3, 3>(6, 6) = Matrix3d::Identity();
    F_.block<3, 3>(9, 6) = T * Matrix3d::Identity();
    F_.block<3, 3>(9, 9) = Matrix3d::Identity();

    Vector3d sigmas;

    // Compose noise covariance matrix for the linear acceleration part
    sigmas << sigma_x, sigma_y, sigma_z;
    sigmas = sigmas.array().square();
    MatrixXd Q_pos(6, 6);
    Q_pos.block<3, 3>(0, 0) = sigmas.asDiagonal() * (std::pow(T, 3.0) / 3.0);
    Q_pos.block<3, 3>(0, 3) = sigmas.asDiagonal() * (std::pow(T, 2.0) / 2.0);
    Q_pos.block<3, 3>(3, 0) = Q_pos.block<3, 3>(0, 3);
    Q_pos.block<3, 3>(3, 3) = sigmas.asDiagonal() * T;

    // Compose noise covariance matrix for the euler angle rates part
    sigmas << sigma_yaw, sigma_pitch, sigma_roll;
    sigmas = sigmas.array().square();
    MatrixXd Q_ang(6, 6);
    Q_ang.block<3, 3>(3, 3) = sigmas.asDiagonal() * (std::pow(T, 3.0) / 3.0);
    Q_ang.block<3, 3>(3, 0) = sigmas.asDiagonal() * (std::pow(T, 2.0) / 2.0);
    Q_ang.block<3, 3>(0, 3) = Q_ang.block<3, 3>(3, 0);
    Q_ang.block<3, 3>(0, 0) = sigmas.asDiagonal() * T;

    Q_ = MatrixXd::Zero(12, 12);
    Q_.block<6, 6>(0, 0) = Q_pos;
    Q_.block<6, 6>(6, 6) = Q_ang;
}


KinematicModel::~KinematicModel()
{ }


std::pair<std::size_t, std::size_t> KinematicModel::getOutputSize() const
{
    // 9 linear components (x, y, z, x_dot, y_dot, z_dot, yaw_dot, pitch_dot, roll_dot)
    // 3 angular components (yaw, pitch, roll)
    return std::make_pair(9, 3);
}


Eigen::MatrixXd KinematicModel::getStateTransitionMatrix()
{
    return F_;
}


Eigen::MatrixXd KinematicModel::getNoiseCovarianceMatrix()
{
    return Q_;
}


bool KinematicModel::setProperty(const std::string& property)
{
    return false;
}

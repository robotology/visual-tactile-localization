/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <Random3DPose.h>

#include <Eigen/Cholesky>

using namespace Eigen;


Random3DPose::Random3DPose
(
    const double T,
    const double sigma_x,  const double sigma_y,  const double sigma_z,
    const double sigma_wx, const double sigma_wy, const double sigma_wz,
    unsigned int seed
) noexcept :
    generator_(std::mt19937_64(seed)),
    distribution_(std::normal_distribution<double>(0.0, 1.0)),
    gauss_rnd_sample_([&] { return (distribution_)(generator_); }),
    T_(T)
{
    Vector3d sigmas;

    /**
     * Compose square root noise covariance matrix for the angular part.
     */
    sigmas << sigma_wx, sigma_wy, sigma_wz;
    sqrt_Q_ang_ = sigmas.asDiagonal() * sqrt(T_);


    /**
     * Compose noise covariance matrix for the linear part.
     */
    sigmas << sigma_x, sigma_y, sigma_z;
    sigmas = sigmas.array().square();
    Matrix<double, 6, 6> Q_pos;
    Q_pos.block<3, 3>(0, 0) = sigmas.asDiagonal() * (std::pow(T_, 3.0) / 3.0);
    Q_pos.block<3, 3>(0, 3) = sigmas.asDiagonal() * (std::pow(T_, 2.0) / 2.0);
    Q_pos.block<3, 3>(3, 0) = Q_pos.block<3, 3>(0, 3);
    Q_pos.block<3, 3>(3, 3) = sigmas.asDiagonal() * T_;


    /**
     * Evaluate the square root matrix of Q_pos.
     */
    LDLT<Matrix<double, 6, 6>> chol_ldlt(Q_pos);
    sqrt_Q_pos_.noalias() = (chol_ldlt.transpositionsP() * Matrix<double, 6, 6>::Identity()).transpose() *
                             chol_ldlt.matrixL() *
                             chol_ldlt.vectorD().real().cwiseSqrt().asDiagonal();
}


Random3DPose::Random3DPose
(
    const double T,
    const double sigma_x,  const double sigma_y,  const double sigma_z,
    const double sigma_wx, const double sigma_wy, const double sigma_wz
) noexcept :
    Random3DPose(T,
                 sigma_x,  sigma_y,  sigma_z,
                 sigma_wx, sigma_wy, sigma_wz,
                 1)
{ }


void Random3DPose::propagate(const Ref<const MatrixXd>& cur_states, Ref<MatrixXd> prop_states)
{
    /**
     * Propagate linear part of the state:
     *
     * x_{k} = x_{k-1} + v_{k-1} * T
     * v_{k} = v_{k-1}
     */
    prop_states.topRows(3).noalias() = cur_states.topRows(3) + T_ * cur_states.middleRows(3, 3);
    prop_states.middleRows(3, 3)     = cur_states.middleRows(3, 3);

    /**
     * Propagate orientation:
     * (Warning: assumes that omega(t) remains constant during T)
     *
     * q_{k} = exp(0.5 * S(omega) * T) * q_{k-1}
     *       = exp(S(0.5 * omega * T)) * q_{k-1}
     *       = exp(S(omega'))          * q_{k-1}
     * with S the 4x4 skew symmetric operator
     *
     * exp( S(omega') = cos(norm(omega')) * I +
     *                  sin(norm(omega')) / norm(omega') * S(omega')
     */

    for (std::size_t i = 0; i < cur_states.cols(); i++)
    {
        Matrix4d S_omega;
        const Ref<const VectorXd> w = cur_states.bottomRows(3).col(i);
        S_omega <<     0, -w(0), -w(1), -w(2),
                    w(0),     0,  w(2), -w(1),
                    w(1), -w(2),     0,  w(0),
                    w(2),  w(1),  -w(0),    0;
        S_omega *= 0.5 * T_;

        double norm_w = (w * 0.5 * T_).norm();

        /* This is to avoid sin(0) / 0 being calculated as -nan. */
        norm_w += std::numeric_limits<double>::min();

        prop_states.middleRows(6, 4).col(i).noalias() =
            (cos(norm_w) * Matrix4d::Identity() + sin(norm_w) / norm_w * S_omega) *
            cur_states.middleRows(6, 4).col(i);
    }

    /**
     * Propagate angular velocity
     */
    prop_states.bottomRows(3) = cur_states.bottomRows(3);
}


void Random3DPose::motion(const Ref<const MatrixXd>& cur_states, Ref<MatrixXd> prop_states)
{
    propagate(cur_states, prop_states);

    MatrixXd noise_samples = getNoiseSample(prop_states.cols());

    prop_states.topRows(6)       += noise_samples.topRows(6);
 // prop_states.middlecols(6, 4) += 0 (no noise enters directly in the quaternion)
    prop_states.bottomRows(3)    += noise_samples.bottomRows(3);
}


MatrixXd Random3DPose::getNoiseSample(const std::size_t num)
{
    MatrixXd rand_vectors(6 + 3, num);
    for (int i = 0; i < rand_vectors.size(); i++)
        *(rand_vectors.data() + i) = gauss_rnd_sample_();

    MatrixXd noise(6 + 3, num);
    noise.topRows(6)    = sqrt_Q_pos_ * rand_vectors.topRows(6);
    noise.bottomRows(3) = sqrt_Q_ang_ * rand_vectors.bottomRows(3);

    return noise;
}


std::pair<std::size_t, std::size_t> Random3DPose::getOutputSize() const
{
    return std::make_pair(13, 0);
}

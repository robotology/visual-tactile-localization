/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef INITPARTICLES_H
#define INITPARTICLES_H

#include <BayesFilters/ParticleSet.h>
#include <BayesFilters/ParticleSetInitialization.h>

#include <Eigen/Dense>

#include <random>

class InitParticles : public bfl::ParticleSetInitialization
{
public:
    InitParticles
    (
        const Eigen::Ref<const Eigen::VectorXd>& center,
        const Eigen::Ref<const Eigen::VectorXd>& radius,
        const Eigen::Ref<const Eigen::MatrixXd>& initial_covariance
    );

    InitParticles
    (
        const unsigned int seed,
        const Eigen::Ref<const Eigen::VectorXd>& center,
        const Eigen::Ref<const Eigen::VectorXd>& radius,
        const Eigen::Ref<const Eigen::MatrixXd>& initial_covariance
    );

    virtual ~InitParticles() noexcept { };

    bool initialize(bfl::ParticleSet& particles) override;

protected:
    Eigen::MatrixXd initial_covariance_;

    std::mt19937_64 generator_;

    std::uniform_real_distribution<double> uniform_x_;

    std::uniform_real_distribution<double> uniform_y_;

    std::uniform_real_distribution<double> uniform_z_;

    std::uniform_real_distribution<double> uniform_yaw_;

    std::uniform_real_distribution<double> uniform_pitch_;

    std::uniform_real_distribution<double> uniform_roll_;

    std::function<double()> uniform_gen_x_;

    std::function<double()> uniform_gen_y_;

    std::function<double()> uniform_gen_z_;

    std::function<double()> uniform_gen_yaw_;

    std::function<double()> uniform_gen_pitch_;

    std::function<double()> uniform_gen_roll_;
};


#endif /* INITPARTICLES_H */

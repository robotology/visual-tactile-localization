/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef INITPARTICLESGROUNDTRUTH_H
#define INITPARTICLESGROUNDTRUTH_H

#include <BayesFilters/ParticleSet.h>
#include <BayesFilters/ParticleSetInitialization.h>

#include <Eigen/Dense>


class InitParticlesGroundTruth : public bfl::ParticleSetInitialization
{
public:
    InitParticlesGroundTruth(const std::string& path, const std::string& object_name, const Eigen::Ref<const Eigen::MatrixXd>& initial_covariance);

    virtual ~InitParticlesGroundTruth() noexcept;

    bool initialize(bfl::ParticleSet& particles) override;

protected:
    std::pair<bool, Eigen::MatrixXd> readStateFromFile(const std::string& filename, const std::size_t num_fields);

    Eigen::VectorXd initial_pose_;

    Eigen::MatrixXd initial_covariance_;

    std::string log_ID_ = "[InitParticlesGroundTruth]";
};

#endif /* INITPARTICLESGROUNDTRUTH_H */

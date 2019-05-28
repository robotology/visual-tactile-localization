/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef PARTICLESCORRECTION_H
#define PARTICLESCORRECTION_H

#include <BayesFilters/LikelihoodModel.h>
#include <BayesFilters/MeasurementModel.h>
#include <BayesFilters/ParticleSet.h>
#include <BayesFilters/PFCorrection.h>
#include <BayesFilters/StateModel.h>

#include <Correction.h>
#include <ProximityLikelihood.h>

#include <functional>
#include <memory>
#include <random>


class ParticlesCorrection : public bfl::PFCorrection
{
public:
    ParticlesCorrection(std::unique_ptr<Correction> gaussian_correction, std::unique_ptr<ProximityLikelihood> likihood_model/*, std::unique_ptr<bfl::StateModel> state_model*/, const bool& sample_from_mean) noexcept;

    ParticlesCorrection(std::unique_ptr<Correction> gaussian_correction, std::unique_ptr<ProximityLikelihood> likelihood_model/*, std::unique_ptr<bfl::StateModel> state_model*/, const bool& sample_from_mean, unsigned int seed) noexcept;

    ParticlesCorrection(ParticlesCorrection&& particles_correction) noexcept;

    virtual ~ParticlesCorrection() noexcept;

    void setLikelihoodModel(std::unique_ptr<bfl::LikelihoodModel> likelihood_model) override;

    void setMeasurementModel(std::unique_ptr<bfl::MeasurementModel> measurement_model) override;

    bfl::MeasurementModel& getMeasurementModel() override;

    bfl::LikelihoodModel& getLikelihoodModel() override;

    std::pair<bool, Eigen::VectorXd> getLikelihood() override;

protected:
    void correctStep(const bfl::ParticleSet& pred_particles, bfl::ParticleSet& corr_particles) override;

    Eigen::VectorXd sampleFromProposal(const Eigen::VectorXd& mean, const Eigen::MatrixXd& covariance);

    double evaluateProposal(const Eigen::VectorXd& state, const Eigen::VectorXd& mean, const Eigen::MatrixXd& covariance);

    std::unique_ptr<Correction> gaussian_correction_;

    std::unique_ptr<ProximityLikelihood> likelihood_model_;

    /**
     * The state model is required to evaluate the Markov transition probability
     * that is used in the update of the particles weight.
     */
    /* std::unique_ptr<bfl::StateModel> state_model_; */

    std::mt19937_64 generator_;

    std::normal_distribution<double> distribution_;

    /**
     * Random number generator function from a Normal distribution.
     * A call to `gaussian_random_sample_()` returns a double-precision floating-point random number.
     */
    std::function<double()> gaussian_random_sample_;

    bool valid_likelihood_;

    Eigen::VectorXd likelihood_;

    bool sample_from_mean_;
};

#endif /* PARTICLESCORRECTION_H */

/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef SIMULATEDPOINTCLOUD_H
#define SIMULATEDPOINTCLOUD_H

#include <Eigen/Dense>

#include <MeshImporter.h>
#include <PointCloudModel.h>
#include <VCGTriMesh.h>

#include <BayesFilters/SimulatedStateModel.h>

#include <random>
#include <unordered_map>


class SimulatedPointCloud : public PointCloudModel, MeshImporter
{
public:
    SimulatedPointCloud
    (
        const std::string& mesh_filename,
        std::unique_ptr<PointCloudPrediction> prediction,
        std::unique_ptr<bfl::SimulatedStateModel> simulated_model,
        const Eigen::Ref<const Eigen::MatrixXd>& model_noise_covariance,
        const Eigen::Ref<const Eigen::VectorXd>& observer_origin,
        const std::size_t number_of_points,
        const bool enable_back_culling
    );

    SimulatedPointCloud
    (
        const std::string& mesh_filename,
        std::unique_ptr<PointCloudPrediction> prediction,
        std::unique_ptr<bfl::SimulatedStateModel> simulated_model,
        const Eigen::Ref<const Eigen::MatrixXd>& model_noise_covariance,
        const Eigen::Ref<const Eigen::VectorXd>& observer_origin,
        const std::size_t number_of_points,
        const bool enable_back_culling,
        unsigned int seed
    );

    void enableNoise(const double std_noise_x, const double std_noise_y, const double std_noise_z);

    bool freeze(const bfl::Data& data = bfl::Data()) override;

    std::pair<bool, bfl::Data> measure(const bfl::Data& data = bfl::Data()) const override;

    std::pair<std::size_t, std::size_t> getOutputSize() const;

    bool prefetchMeasurements(const std::size_t number_of_steps);

    Eigen::VectorXd getNoiseSample(const std::size_t number);

protected:
    void transformModel(const Eigen::VectorXd& state, simpleTriMesh& transformed);

    Eigen::VectorXd samplePointCloud(const Eigen::VectorXd& state);

    bool noisy_;

    bool enable_back_culling_;

    Eigen::VectorXd observer_origin_;

    Eigen::MatrixXd measurement_;

    Eigen::Matrix3d sqrt_noise_covariance_;

    simpleTriMesh trimesh_;

    std::unique_ptr<bfl::SimulatedStateModel> simulated_model_;

    std::size_t number_of_points_;

    std::size_t step_;

    std::unordered_map<std::size_t, Eigen::MatrixXd> fetched_measurements_;

    std::mt19937_64 generator_;

    std::normal_distribution<float> distribution_;

    std::function<float()> gauss_rnd_sample_;
};

#endif /* SIMULATEDPOINTCLOUD_H */

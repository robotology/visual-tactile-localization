/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef NANOFLANNPOINTCLOUDPREDICTION_H
#define NANOFLANNPOINTCLOUDPREDICTION_H

#include <Eigen/Dense>

#include <ObjectSampler.h>
#include <PointCloudPrediction.h>
#include <kdTree.h>
#include <kdTreeWithNormals.h>

#include <BayesFilters/ParticleSet.h>
#include <BayesFilters/ParticleSetInitialization.h>

#include <yarp/os/Port.h>

#include <memory>
#include <string>

#include <thrift/ModelIDL.h>


class NanoflannPointCloudPrediction : public PointCloudPrediction,
                                      public ModelIDL
{
public:
    NanoflannPointCloudPrediction(std::unique_ptr<ObjectSampler> obj_sampler, const std::size_t number_of_points, const bool& use_normals = false);

    ~NanoflannPointCloudPrediction();

    bool init() override;

    std::pair<bool, Eigen::MatrixXd> predictPointCloud(ConstMatrixXdRef state, ConstVectorXdRef meas) override;

    std::pair<bool, Eigen::MatrixXd> evaluateDistances(ConstMatrixXdRef state, ConstVectorXdRef meas) override;

    Eigen::MatrixXd evaluateModel(const Eigen::Transform<double, 3, Eigen::Affine>& object_pose);

    /* Thrift interface */
    bool initialize_model(const std::string& object_name) override;

    bool enable_normals(const bool enable) override;

private:
    std::unique_ptr<ObjectSampler> obj_sampler_;

    std::size_t number_of_points_;

    Eigen::MatrixXd cloud_;

    Eigen::MatrixXd cloud_with_normals_;

    std::unique_ptr<PointCloudAdaptor> adapted_cloud_;

    std::unique_ptr<PointCloudAdaptor> adapted_cloud_with_normals_;

    std::unique_ptr<kdTree> tree_;

    std::unique_ptr<kdTreeWithNormals> tree_with_normals_;

    bool use_normals_;

    yarp::os::Port port_rpc_command_;

    const std::string log_ID_ = "NanoflannPointCloudPrediction";
};

#endif

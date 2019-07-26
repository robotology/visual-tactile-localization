/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef OBJECTPOINTCLOUDPREDICTION_H
#define OBJECTPOINTCLOUDPREDICTION_H

#include <Eigen/Dense>

#include <ObjectSampler.h>
#include <PointCloudPrediction.h>
#include <kdTree.h>

#include <BayesFilters/ParticleSet.h>
#include <BayesFilters/ParticleSetInitialization.h>

#include <yarp/os/Port.h>

#include <memory>
#include <string>

#include <thrift/ModelIDL.h>


class ObjectPointCloudPrediction : public PointCloudPrediction,
                                   public ModelIDL
{
public:
    ObjectPointCloudPrediction(std::unique_ptr<ObjectSampler> obj_sampler, const std::size_t number_of_points, const bool& use_normals = false);

    ~ObjectPointCloudPrediction();

    bool init() override;

    std::pair<bool, Eigen::MatrixXd> predictPointCloud(ConstMatrixXdRef state, ConstVectorXdRef meas, const bool& enforce_no_normals) override;

    std::pair<bool, Eigen::MatrixXd> evaluateDistances(ConstMatrixXdRef state, ConstVectorXdRef meas) override;

    /* Thrift interface */
    bool initialize_model(const std::string& object_name);

    bool enable_normals(const bool enable);

private:
    std::unique_ptr<ObjectSampler> obj_sampler_;

    std::size_t number_of_points_;

    Eigen::MatrixXd cloud_;

    yarp::os::Port port_rpc_command_;

    const std::string log_ID_ = "ObjectPointCloudPrediction";
};

#endif

/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef LOCALIZESUPERQUADRIC_H
#define LOCALIZESUPERQUADRIC_H

#include <Camera.h>
#include <ObjectSampler.h>
#include <MaskSegmentation.h>

#include <Eigen/Dense>

#include <BayesFilters/ParticleSetInitialization.h>

#include <memory>
#include <string>

#include <vtkSmartPointer.h>
#include <vtkSuperquadric.h>

#include <yarp/os/RpcClient.h>
#include <yarp/sig/PointCloud.h>


class LocalizeSuperquadricSampler : public ObjectSampler
{
public:
    LocalizeSuperquadricSampler(const std::string& port_prefix, const std::string& camera_name, const std::string& camera_fallback_key, const std::string& camera_laterality = "");

    virtual ~LocalizeSuperquadricSampler();

    std::pair<bool, Eigen::MatrixXd> sample(const std::size_t& number_of_points) override;

    void setObjectName(const std::string& object_name) override;

    Eigen::VectorXd getObjectPose();

    std::unique_ptr<bfl::ParticleSetInitialization> getParticleSetInitialization();

protected:
    std::pair<bool, vtkSmartPointer<vtkSuperquadric>> getSuperquadricFromRpc(const yarp::sig::PointCloudXYZRGBA& yarp_point_cloud);

    std::string getObjectMaskName(const std::string& object_name);

    std::unique_ptr<MaskSegmentation> segmentation_;

    std::unique_ptr<Camera> camera_;

    Eigen::VectorXd object_pose_;

    yarp::os::RpcClient localize_superq_rpc_client_;

    const std::string log_ID_ = "LocalizeSuperquadricSampler";

    std::string object_name_;
};


class LocalizeSuperquadricInitializer : public bfl::ParticleSetInitialization
{
public:
    LocalizeSuperquadricInitializer(LocalizeSuperquadricSampler& sampler);

    virtual ~LocalizeSuperquadricInitializer();

    bool initialize(bfl::ParticleSet& particles) override;

protected:
    LocalizeSuperquadricSampler& sampler_;
};

#endif

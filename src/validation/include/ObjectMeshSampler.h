/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef OBJECTMESHSAMPLER_H
#define OBJECTMESHSAMPLER_H

#include <ObjectSampler.h>
#include <VCGTriMesh.h>

#include <string>

class ObjectMeshSampler : public ObjectSampler
{
public:
    ObjectMeshSampler(const std::string& file_name);

    virtual ~ObjectMeshSampler();

    std::pair<bool, Eigen::MatrixXd> sample(const std::size_t& number_of_points) override;

protected:
    simpleTriMesh trimesh_;

    const std::string log_ID_ = "[ObjectMeshSampler]";
};

#endif

/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifdef USE_SUPERQUADRICLIB

#ifndef SUPERQUADRIC_H
#define SUPERQUADRIC_H

#include <ObjectSampler.h>

#include <SuperquadricLibModel/superquadric.h>

#include <string>

#include <yarp/os/RpcClient.h>


class SuperquadricSampler : public ObjectSampler
{
public:
    SuperquadricSampler(const std::string& port_prefix);

    SuperquadricSampler(SuperqModel::Superquadric& superquadric);

    virtual ~SuperquadricSampler();

    std::pair<bool, Eigen::MatrixXd> sample(const std::size_t& number_of_points) override;

protected:
    bool getSuperquadricFromRpc();

    bool get_superq_from_rpc_ = false;

    SuperqModel::Superquadric superquadric_;

    yarp::os::RpcClient superq_rpc_client_;

    const std::string log_ID_ = "SuperquadricSampler";
};

#endif

#endif

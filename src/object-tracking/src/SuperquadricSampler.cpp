/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifdef USE_SUPERQUADRICLIB

#include <SuperquadricSampler.h>

// This is not on https://github.com/robotology/superquadric-lib yet
// #include <SuperquadricLibModel/superquadricSampler.h>

using namespace Eigen;
using namespace SuperqModel;


SuperquadricSampler::SuperquadricSampler(Superquadric& superquadric) :
    superquadric_(superquadric)
{ }


SuperquadricSampler::SuperquadricSampler(const std::string& port_prefix) :
    get_superq_from_rpc_(true)
{
    if (!(superq_rpc_client_.open("/" + port_prefix + "/superquadric_rpc:o")))
    {
        std::string err = log_ID_ + "::ctor. Error: cannot open superquadric rpc client port.";
        throw(std::runtime_error(err));
    }
}


SuperquadricSampler::~SuperquadricSampler()
{
    if (get_superq_from_rpc_)
        superq_rpc_client_.close();
}


std::pair<bool, MatrixXd> SuperquadricSampler::sample(const std::size_t& number_of_points)
{
    if (get_superq_from_rpc_)
    {
        if (!getSuperquadricFromRpc())
            return std::make_pair(false, MatrixXd());
    }

    // This is not on https://github.com/robotology/superquadric-lib yet
    // UniformSampler sampler(0.05);
    // MatrixXd cloud = sampler.sample(superquadric_, true);
    // TODO: add code for poisson-disk sampling or voxel-grid filtering here
    // return std::make_pair(true, cloud);
    std::string err = log_ID_ + "::sample. Error: superquadric sampling not yet implemented.";
    throw(std::runtime_error(err));
}


bool SuperquadricSampler::getSuperquadricFromRpc()
{
    std::string err = log_ID_ + "::sample. Error: superquadric retrieval from rpc not yet implemented.";
    throw(std::runtime_error(err));
}

#endif

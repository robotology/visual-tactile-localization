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

#include <BayesFilters/ParticleSet.h>
#include <BayesFilters/ParticleSetInitialization.h>

#include <nanoflann.hpp>

#include <yarp/os/Port.h>

#include <memory>
#include <string>

#include <thrift/ModelInitializationIDL.h>


// Adapted from nanoflann examples
struct PointCloudAdaptor
{
    const Eigen::Ref<const Eigen::MatrixXd> data;

    PointCloudAdaptor(const Eigen::Ref<const Eigen::MatrixXd>& data_) : data(data_) { }

    /// CRTP helper method
    inline Eigen::Ref<const Eigen::MatrixXd> derived() const { return data; }

    // Must return the number of data points
    inline std::size_t kdtree_get_point_count() const { return data.cols(); }

    // Returns the dim'th component of the idx'th point in the class:
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        return derived()(dim, idx);
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }
};

using kdTree = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, PointCloudAdaptor > ,
                                                   PointCloudAdaptor,
                                                   3 /* dimension, since using point clouds */>;

class NanoflannPointCloudPrediction : public PointCloudPrediction,
                                      public ModelInitializationIDL
{
public:
    NanoflannPointCloudPrediction(std::unique_ptr<ObjectSampler> obj_sampler, const std::size_t number_of_points);

    ~NanoflannPointCloudPrediction();

    bool init() override;

    std::pair<bool, Eigen::MatrixXd> predictPointCloud(ConstMatrixXdRef state, ConstVectorXdRef meas) override;

    std::pair<bool, Eigen::MatrixXd> evaluateDistances(ConstMatrixXdRef state, ConstVectorXdRef meas) override;

    /* Thrift interface */
    bool initialize_model(const std::string& object_name);

protected:
    std::unique_ptr<ObjectSampler> obj_sampler_;

    std::size_t number_of_points_;

    Eigen::MatrixXd cloud_;

    std::unique_ptr<PointCloudAdaptor> adapted_cloud_;

    std::unique_ptr<kdTree> tree_;

    yarp::os::Port port_rpc_command_;

    const std::string log_ID_ = "NanoflannPointCloudPrediction";
};

#endif

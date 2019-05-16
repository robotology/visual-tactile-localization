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

#include <thrift/ModelIDL.h>


// Metric to be used with positions and normals
template <class T, class DataSource, typename _DistanceType = T>
struct PositionWithNormal_Adaptor
{
    typedef T ElementType;
    typedef _DistanceType DistanceType;

    const DataSource &data_source;
    DistanceType weight_position = 1.0;
    DistanceType weight_phi = 0.0;
    DistanceType weight_lambda = 0.0;

    PositionWithNormal_Adaptor(const DataSource &_data_source) :
        data_source(_data_source) {}

    void setWeights(const DistanceType &_weight_position, const DistanceType &_weight_phi, const DistanceType &_weight_lambda)
    {
        weight_position = _weight_position;
        weight_phi = _weight_phi;
        weight_lambda = _weight_lambda;
    }

    inline DistanceType evalMetric(const T *a, const size_t b_idx,
                                 size_t size) const
    {
        DistanceType result = DistanceType(), PI = nanoflann::pi_const<DistanceType>();
        const DistanceType diff_x = a[0] - data_source.kdtree_get_pt(b_idx, 0);
        const DistanceType diff_y = a[1] - data_source.kdtree_get_pt(b_idx, 1);
        const DistanceType diff_z = a[2] - data_source.kdtree_get_pt(b_idx, 2);
        DistanceType diff_phi = a[3] - data_source.kdtree_get_pt(b_idx, 3);
        DistanceType diff_lambda = a[4] - data_source.kdtree_get_pt(b_idx, 4);
        if (diff_phi > PI)
            diff_phi -= 2 * PI;
        else if (diff_phi < -PI)
            diff_phi += 2 * PI;
        if (diff_lambda > PI)
            diff_lambda -= 2 * PI;
        else if (diff_lambda < -PI)
            diff_lambda += 2 * PI;

        result += weight_position * (diff_x * diff_x + diff_y * diff_y + diff_z * diff_z) +
            weight_phi * (diff_phi * diff_phi) +
            weight_lambda * (diff_lambda * diff_lambda);

        return result;
    }

    template <typename U, typename V>
    inline DistanceType accum_dist(const U a, const V b, const size_t i) const
    {
        DistanceType result = DistanceType(), PI = nanoflann::pi_const<DistanceType>();
        result = (a - b) * (a - b);

        if (i >= 3 && result > PI)
            result -= 2 * PI;
        else if (i >= 3 && result < -PI)
            result += 2 * PI;

        if (i < 3)
            result *= weight_position;
        else if (i == 3)
            result *= weight_phi;
        else
            result *= weight_lambda;
        return result;
    }
};


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

using kdTree = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, PointCloudAdaptor >,
                                                   PointCloudAdaptor,
                                                   3 /* dimension, since using point clouds */>;

using kdTreeWithNormals = nanoflann::KDTreeSingleIndexAdaptor<PositionWithNormal_Adaptor<double, PointCloudAdaptor >,
                                                              PointCloudAdaptor,
                                                              5 /* dimension, since using point clouds including normals */>;

class NanoflannPointCloudPrediction : public PointCloudPrediction,
                                      public ModelIDL
{
public:
    NanoflannPointCloudPrediction(std::unique_ptr<ObjectSampler> obj_sampler, const std::size_t number_of_points, const bool& use_normals = false);

    ~NanoflannPointCloudPrediction();

    bool init() override;

    std::pair<bool, Eigen::MatrixXd> predictPointCloud(ConstMatrixXdRef state, ConstVectorXdRef meas) override;

    std::pair<bool, Eigen::MatrixXd> evaluateDistances(ConstMatrixXdRef state, ConstVectorXdRef meas) override;

    /* Thrift interface */
    bool initialize_model(const std::string& object_name);

    bool enable_normals(const bool enable);

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

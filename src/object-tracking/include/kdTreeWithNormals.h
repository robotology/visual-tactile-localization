/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef KDTREEWITHNORMALS_H
#define KDTREEWITHNORMALS_H

#include <PointCloudAdaptor.h>

#include <nanoflann.hpp>

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

using kdTreeWithNormals = nanoflann::KDTreeSingleIndexAdaptor<PositionWithNormal_Adaptor<double, PointCloudAdaptor >,
                                                              PointCloudAdaptor,
                                                              5 /* dimension, since using point clouds including normals */>;
#endif

/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <NanoflannPointCloudPrediction.h>
#ifdef _OPENMP
#include <omp.h>
#endif

using namespace Eigen;
using namespace nanoflann;


NanoflannPointCloudPrediction::NanoflannPointCloudPrediction(std::unique_ptr<ObjectSampler> obj_sampler, std::size_t number_of_points) :
    obj_sampler_(std::move(obj_sampler)),
    number_of_points_(number_of_points)
{ }


bool NanoflannPointCloudPrediction::init()
{
    // Sample the point cloud once
    bool valid_cloud = false;
    std::tie(valid_cloud, cloud_) = obj_sampler_->sample(number_of_points_);
    if (!valid_cloud)
        return false;

    // Initialize tree
    adapted_cloud_ = std::unique_ptr<PointCloudAdaptor>(new PointCloudAdaptor(cloud_));
    tree_ = std::unique_ptr<kdTree>(new kdTree(3 /* dim */, *adapted_cloud_, KDTreeSingleIndexAdaptorParams(10 /* max leaf */)));
    tree_->buildIndex();

    return true;
}


bool NanoflannPointCloudPrediction::reset()
{
    return init();
}


std::pair<bool, MatrixXd> NanoflannPointCloudPrediction::predictPointCloud(ConstMatrixXdRef state, ConstVectorXdRef meas)
{
    // Check if meas size is multiple of 3
    if ((meas.size() % 3) != 0)
        return std::make_pair(false, MatrixXd(0, 0));

    std::size_t components = meas.size() / 3;
    MatrixXd predictions(meas.size(), state.cols());

    // Reshape measurement as a matrix
    Map<const MatrixXd> meas_matrix(meas.data(), 3, components);

    // Process all the states
    MatrixXd meas_body(3, components * state.cols());
    std::vector<Transform<double, 3, Eigen::Affine>> poses(state.cols());
    #pragma omp parallel for
    for (std::size_t i = 0; i < state.cols(); i++)
    {
        // const Ref<const VectorXd> state_i = state.col(i);

        // Transform<double, 3, Eigen::Affine> pose;

        // Compose translation
        poses[i] = Translation<double, 3>(state.col(i).segment(0, 3));

        // Compose rotation
        poses[i].rotate(AngleAxis<double>(state(9,  i), Vector3d::UnitZ()));
        poses[i].rotate(AngleAxis<double>(state(10, i), Vector3d::UnitY()));
        poses[i].rotate(AngleAxis<double>(state(11, i), Vector3d::UnitX()));

        // Express measurement in body fixed frame
        meas_body.middleCols(components * i, components) = poses[i].inverse() * meas_matrix.colwise().homogeneous();
    }

    // Predict measurement
    MatrixXd pred_meas_body(3, state.cols() * components);
    #pragma omp parallel for collapse(2)
    for (std::size_t i = 0; i < state.cols(); i++)
    {
        for (std::size_t j = 0; j < components; j++)
        {
            const Ref<const Vector3d> meas_j = meas_body.middleCols(components * i, components).col(j);

            std::size_t ret_index;
            double out_dist_sqr;
            KNNResultSet<double> resultSet(1);
            resultSet.init(&ret_index, &out_dist_sqr);
            // Querying tree_ is thread safe as per this issue
            // https://github.com/jlblancoc/nanoflann/issues/54
            tree_->findNeighbors(resultSet, meas_j.data(), nanoflann::SearchParams(10));

            pred_meas_body.middleCols(components * i, components).col(j) = cloud_.col(ret_index);
        }
    }

    #pragma omp parallel for
    for (std::size_t i = 0; i < state.cols(); i++)
    {
        // Convert back in robot frame
        MatrixXd meas_robot = poses[i] * pred_meas_body.middleCols(components * i, components).colwise().homogeneous();

        // Store predicted measurement
        predictions.col(i) = std::move(Map<VectorXd>(meas_robot.data(), meas.size()));
    }

    return std::make_pair(true, predictions);
}


std::pair<bool, MatrixXd> NanoflannPointCloudPrediction::evaluateDistances(ConstMatrixXdRef state, ConstVectorXdRef meas)
{
    // Check if meas size is multiple of 3
    if ((meas.size() % 3) != 0)
        return std::make_pair(false, MatrixXd(0, 0));

    std::size_t components = meas.size() / 3;
    MatrixXd squared_distances(state.cols(), components);

    // Reshape measurement as a matrix
    Map<const MatrixXd> meas_matrix(meas.data(), 3, components);

    // Process all the states
    MatrixXd meas_body(3, components * state.cols());
    std::vector<Transform<double, 3, Eigen::Affine>> poses(state.cols());
    #pragma omp parallel for
    for (std::size_t i = 0; i < state.cols(); i++)
    {
        // Compose translation
        poses[i] = Translation<double, 3>(state.col(i).segment(0, 3));

        // Compose rotation
        poses[i].rotate(AngleAxis<double>(state(9,  i), Vector3d::UnitZ()));
        poses[i].rotate(AngleAxis<double>(state(10, i), Vector3d::UnitY()));
        poses[i].rotate(AngleAxis<double>(state(11, i), Vector3d::UnitX()));

        // Express measurement in body fixed frame
        meas_body.middleCols(components * i, components) = poses[i].inverse() * meas_matrix.colwise().homogeneous();
    }

    // Eval distances
    #pragma omp parallel for collapse(2)
    for (std::size_t i = 0; i < state.cols(); i++)
    {
        for (std::size_t j = 0; j < components; j++)
        {
            const Ref<const Vector3d> meas_j = meas_body.middleCols(components * i, components).col(j);

            std::size_t ret_index;
            double out_dist_sqr;
            KNNResultSet<double> resultSet(1);
            resultSet.init(&ret_index, &out_dist_sqr);
            // Querying tree_ is thread safe as per this issue
            // https://github.com/jlblancoc/nanoflann/issues/54
            tree_->findNeighbors(resultSet, meas_j.data(), nanoflann::SearchParams(10));

            squared_distances(i, j) = out_dist_sqr;
        }
    }

    return std::make_pair(true, squared_distances);
}

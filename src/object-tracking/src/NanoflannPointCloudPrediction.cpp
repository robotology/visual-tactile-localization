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

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

using namespace Eigen;
using namespace nanoflann;
using namespace pcl;


NanoflannPointCloudPrediction::NanoflannPointCloudPrediction(std::unique_ptr<ObjectSampler> obj_sampler, std::size_t number_of_points, const bool& use_normals) :
    obj_sampler_(std::move(obj_sampler)),
    number_of_points_(number_of_points),
    use_normals_(use_normals)
{
    // Open RPC input port for commands
    if (!port_rpc_command_.open("/object-tracking/modelling/cmd:i"))
    {
        std::string err = log_ID_ + "::ctor. Error: cannot open RPC input command port.";
        throw(std::runtime_error(err));
    }

    if (!(this->yarp().attachAsServer(port_rpc_command_)))
    {
        std::string err = "PFILTER::CTOR::ERROR\n\tError: cannot attach the RPC command port.";
        throw(std::runtime_error(err));
    }
}


NanoflannPointCloudPrediction::~NanoflannPointCloudPrediction()
{
    port_rpc_command_.close();
}


bool NanoflannPointCloudPrediction::init()
{
    // Sample the point cloud once
    bool valid_cloud = false;
    std::tie(valid_cloud, cloud_with_normals_) = obj_sampler_->sample(number_of_points_);
    if (!valid_cloud)
        return false;

    // Cloud without normals
    cloud_ = cloud_with_normals_.topRows<3>();

    // Initialize tree
    adapted_cloud_ = std::unique_ptr<PointCloudAdaptor>(new PointCloudAdaptor(cloud_));
    tree_ = std::unique_ptr<kdTree>(new kdTree(3 /* dim */, *adapted_cloud_, KDTreeSingleIndexAdaptorParams(10 /* max leaf */)));
    tree_->buildIndex();

    if (use_normals_)
    {
        // Initialize tree with normals
        double weight_position = 1.0;
        double weight_phi = 0.001;
        double weight_lambda = 0.001;

        adapted_cloud_with_normals_ = std::unique_ptr<PointCloudAdaptor>(new PointCloudAdaptor(cloud_with_normals_));
        tree_with_normals_ = std::unique_ptr<kdTreeWithNormals>(new kdTreeWithNormals(5/* dim */, *adapted_cloud_with_normals_, KDTreeSingleIndexAdaptorParams(10 /* max leaf */)));
        tree_with_normals_->distance.setWeights(weight_position, weight_phi, weight_lambda);
        tree_with_normals_->buildIndex();
    }

    return true;
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

    // If required, evaluate normals
    MatrixXd normals(3, components);
    if (use_normals_)
    {
        PointCloud<PointXYZ>::Ptr pcl_cloud (new PointCloud<PointXYZ>);
        PointCloud<Normal>::Ptr pcl_normals (new PointCloud<Normal>);
        search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ>());

        for (std::size_t i = 0; i < components; i++)
        {
            PointXYZ p;
            p.x = meas_matrix(0, i);
            p.y = meas_matrix(1, i);
            p.z = meas_matrix(2, i);

            pcl_cloud->push_back(p);
        }

        NormalEstimationOMP<PointXYZ, Normal> ne;
        ne.setInputCloud(pcl_cloud);
        ne.setSearchMethod(tree);
        ne.setRadiusSearch(0.03);
        ne.compute(*pcl_normals);

        for (std::size_t i = 0; i < components; i++)
            normals.col(i) = Vector3d(pcl_normals->at(i).normal[0], pcl_normals->at(i).normal[1], pcl_normals->at(i).normal[2]);
    }

    // Process all the states
    std::size_t meas_size = use_normals_ ? 5 : 3;
    MatrixXd meas_body(meas_size, components * state.cols());
    MatrixXd normals_body(3, components * state.cols());
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
        meas_body.middleCols(components * i, components).topRows<3>() = poses[i].inverse() * meas_matrix.colwise().homogeneous();
        if (use_normals_)
            normals_body.middleCols(components * i, components) = poses[i].rotation().transpose() * normals;
    }

    if (use_normals_)
    {
#pragma omp parallel for
        for (std::size_t i = 0; i < components * state.cols(); i++)
        {
            meas_body(3, i) = std::atan2(normals_body(2, i), std::sqrt(normals_body(0, i) * normals_body(0, i) + normals_body(1, i) * normals_body(1, i)));
            meas_body(4, i) = std::atan2(normals_body(1, i), normals_body(0, i));
        }
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
            if (use_normals_)
                tree_with_normals_->findNeighbors(resultSet, meas_j.transpose().data(), nanoflann::SearchParams(10));
            else
                tree_->findNeighbors(resultSet, meas_j.data(), nanoflann::SearchParams(10));

            // In case the NN search fails, we set predicted measurements equal to the measurements
            // in order to gracefully handle this exception
            if (ret_index > cloud_with_normals_.cols())
                pred_meas_body.middleCols(components * i, components).col(j) = meas_j.head<3>();
            else
            {
                if (use_normals_)
                    pred_meas_body.middleCols(components * i, components).col(j) = cloud_with_normals_.col(ret_index).head<3>();
                else
                    pred_meas_body.middleCols(components * i, components).col(j) = cloud_.col(ret_index);
            }
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


bool NanoflannPointCloudPrediction::initialize_model(const std::string& object_name)
{
    // Set the object model within the object sampler class
    obj_sampler_->setObjectName(object_name);

    // Initialize the class
    init();

    return true;
}

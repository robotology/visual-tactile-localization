/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ObjectPointCloudPrediction.h>
#ifdef _OPENMP
#include <omp.h>
#endif

using namespace Eigen;
using namespace nanoflann;


ObjectPointCloudPrediction::ObjectPointCloudPrediction(std::unique_ptr<ObjectSampler> obj_sampler, std::size_t number_of_points, const bool& use_normals) :
    obj_sampler_(std::move(obj_sampler)),
    number_of_points_(number_of_points)
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


ObjectPointCloudPrediction::~ObjectPointCloudPrediction()
{
    port_rpc_command_.close();
}


bool ObjectPointCloudPrediction::init()
{
    // Sample the point cloud once
    bool valid_cloud = false;
    std::tie(valid_cloud, cloud_) = obj_sampler_->sample(number_of_points_);
    if (!valid_cloud)
        return false;

    return true;
}


std::pair<bool, MatrixXd> ObjectPointCloudPrediction::predictPointCloud(ConstMatrixXdRef state, ConstVectorXdRef meas)
{
    // Check if meas size is multiple of 3
    if ((meas.size() % 3) != 0)
        return std::make_pair(false, MatrixXd(0, 0));

    // Reshape measurement as a matrix
    Map<const MatrixXd> meas_matrix(meas.data(), 3, meas.size() / 3);

    // Move sampled point cloud to robot frame according to states stored in state
    std::vector<MatrixXd> clouds(state.cols());
    std::vector<std::unique_ptr<PointCloudAdaptor>> adapted_clouds(state.cols());
    std::vector<std::unique_ptr<kdTree>> trees(state.cols());
    #pragma omp parallel for
    for (std::size_t i = 0; i < state.cols(); i++)
    {
        // Compose translation
        Transform<double, 3, Eigen::Affine> pose;
        pose = Translation<double, 3>(state.col(i).segment(0, 3));

        // Compose rotation
        AngleAxisd rotation(AngleAxis<double>(state(9,  i), Vector3d::UnitZ()) *
                            AngleAxis<double>(state(10, i), Vector3d::UnitY()) *
                            AngleAxis<double>(state(11, i), Vector3d::UnitX()));
        pose.rotate(rotation);

        // Express point cloud sampled on the model in robot frame
        clouds[i] = pose * cloud_.topRows<3>().colwise().homogeneous();

        // Express normals sampled on the model in robot frame
        // Not used for now
        MatrixXd normals = pose.rotation() * cloud_.bottomRows<3>();

        // Initialize tree
        adapted_clouds[i] = std::unique_ptr<PointCloudAdaptor>(new PointCloudAdaptor(clouds.at(i)));
        trees[i] = std::unique_ptr<kdTree>(new kdTree(3 /* dim */, *adapted_clouds.at(i), KDTreeSingleIndexAdaptorParams(10 /* max leaf */)));
        trees[i]->buildIndex();
    }

    // Predict measurement
    MatrixXd predictions(meas.size(), state.cols());
    #pragma omp parallel for collapse(2)
    for (std::size_t i = 0; i < state.cols(); i++)
    {
        for (std::size_t j = 0; j < meas.size() / 3; j++)
        {
            const Ref<const Vector3d> meas_j = meas_matrix.col(j);

            std::size_t ret_index;
            double out_dist_sqr;
            KNNResultSet<double> resultSet(1);
            resultSet.init(&ret_index, &out_dist_sqr);
            // Querying tree_ is thread safe as per this issue
            // https://github.com/jlblancoc/nanoflann/issues/54
            trees.at(i)->findNeighbors(resultSet, meas_j.data(), nanoflann::SearchParams(10));

            predictions.col(i).segment<3>(j * 3) = clouds.at(i).col(ret_index);
        }
    }

    return std::make_pair(true, predictions);
}


std::pair<bool, MatrixXd> ObjectPointCloudPrediction::evaluateDistances(ConstMatrixXdRef state, ConstVectorXdRef meas)
{
    // Check if meas size is multiple of 3
    if ((meas.size() % 3) != 0)
        return std::make_pair(false, MatrixXd(0, 0));

    MatrixXd squared_distances(state.cols(), meas.size() / 3);

    // Reshape measurement as a matrix
    Map<const MatrixXd> meas_matrix(meas.data(), 3, meas.size() / 3);

    // Move sampled point cloud to robot frame according to states stored in state
    std::vector<MatrixXd> clouds(state.cols());
    std::vector<std::unique_ptr<PointCloudAdaptor>> adapted_clouds(state.cols());
    std::vector<std::unique_ptr<kdTree>> trees(state.cols());
    #pragma omp parallel for
    for (std::size_t i = 0; i < state.cols(); i++)
    {
        // Compose translation
        Transform<double, 3, Eigen::Affine> pose;
        pose = Translation<double, 3>(state.col(i).segment(0, 3));

        // Compose rotation
        AngleAxisd rotation(AngleAxis<double>(state(9,  i), Vector3d::UnitZ()) *
                            AngleAxis<double>(state(10, i), Vector3d::UnitY()) *
                            AngleAxis<double>(state(11, i), Vector3d::UnitX()));
        pose.rotate(rotation);

        // Express point cloud sampled on the model in robot frame
        clouds[i] = pose * cloud_.topRows<3>().colwise().homogeneous();

        // Express normals sampled on the model in robot frame
        // Not used for now
        MatrixXd normals = pose.rotation() * cloud_.bottomRows<3>();

        // Initialize tree
        adapted_clouds[i] = std::unique_ptr<PointCloudAdaptor>(new PointCloudAdaptor(clouds.at(i)));
        trees[i] = std::unique_ptr<kdTree>(new kdTree(3 /* dim */, *adapted_clouds.at(i), KDTreeSingleIndexAdaptorParams(10 /* max leaf */)));
        trees[i]->buildIndex();
    }

    // Eval distances
    #pragma omp parallel for collapse(2)
    for (std::size_t i = 0; i < state.cols(); i++)
    {
        for (std::size_t j = 0; j < meas.size() / 3; j++)
        {
            const Ref<const Vector3d> meas_j = meas_matrix.col(j);

            std::size_t ret_index;
            double out_dist_sqr;
            KNNResultSet<double> resultSet(1);
            resultSet.init(&ret_index, &out_dist_sqr);
            // Querying tree_ is thread safe as per this issue
            // https://github.com/jlblancoc/nanoflann/issues/54
            trees.at(i)->findNeighbors(resultSet, meas_j.data(), nanoflann::SearchParams(10));

            squared_distances(i, j) = out_dist_sqr;
        }
    }

    return std::make_pair(true, squared_distances);
}


bool ObjectPointCloudPrediction::initialize_model(const std::string& object_name)
{
    // Set the object model within the object sampler class
    obj_sampler_->setObjectName(object_name);

    // Initialize the class
    init();

    return true;
}


bool ObjectPointCloudPrediction::enable_normals(const bool enable)
{
    return false;
}

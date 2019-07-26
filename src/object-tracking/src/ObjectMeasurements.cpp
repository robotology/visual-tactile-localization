/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ObjectMeasurements.h>

#include <mlpack/methods/neighbor_search/ns_model.hpp>
#include <mlpack/methods/approx_kfn/drusilla_select.hpp>

#include <armadillo>
#include <vector>

using namespace bfl;
using namespace Eigen;
using namespace mlpack::neighbor;
using KFNModel = NSModel<FurthestNS>;


ObjectMeasurements::ObjectMeasurements
(
    std::unique_ptr<Camera> camera,
    std::shared_ptr<PointCloudSegmentation> segmentation,
    std::shared_ptr<PointCloudPrediction> prediction,
    const Eigen::Ref<const Eigen::Matrix3d>& visual_noise_covariance,
    const double& visual_outlier_threshold,
    const std::string& depth_fetch_mode,
    const bool& enable_log,
    const std::string& log_path
) :
    camera_(std::move(camera)),
    segmentation_(segmentation),
    prediction_(prediction),
    visual_noise_covariance_(visual_noise_covariance),
    visual_outlier_threshold_(visual_outlier_threshold),
    depth_fetch_mode_(depth_fetch_mode),
    enable_log_(enable_log),
    log_path_(log_path)
{
    if (enable_log_)
        this->enable_log(log_path_, "object-tracking");
}


ObjectMeasurements::ObjectMeasurements
(
    std::unique_ptr<Camera> camera,
    std::shared_ptr<PointCloudSegmentation> segmentation,
    std::shared_ptr<PointCloudPrediction> prediction,
    const Eigen::Ref<const Eigen::Matrix3d>& visual_noise_covariance,
    const Eigen::Ref<const Eigen::Matrix3d>& tactile_noise_covariance,
    const double& visual_outlier_threshold,
    const std::string& depth_fetch_mode,
    const bool& enable_log,
    const std::string& log_path
) :
    ObjectMeasurements(std::move(camera), segmentation, prediction, visual_noise_covariance, visual_outlier_threshold, depth_fetch_mode, enable_log, log_path)
{
    tactile_noise_covariance_ = tactile_noise_covariance;
    has_tactile_noise_covariance_ = true;
}


std::pair<bool, Data> ObjectMeasurements::measure(const Data& data) const
{
    return std::make_pair(true, measurement_);
}

#include <fstream>

bool ObjectMeasurements::freeze()
{
    // Freeze camera
    measurements_available_ = camera_->freeze();
    if (!measurements_available_)
        return false;

    // Get depth image
    if(!getDepth())
        return false;

    // Freeze segmentation
    if (!segmentation_->freezeSegmentation(*camera_))
        return false;

    // Get 3D point cloud.
    bool blocking_call = false;
    bool valid_point_cloud;
    MatrixXd point_cloud;
    MatrixXd point_cloud_tmp;
    std::tie(valid_point_cloud, point_cloud) = segmentation_->extractPointCloud(*camera_, depth_, 10.0);
    if (!valid_point_cloud)
        return false;

    // Predict point cloud with all the measurements
    bool valid_prediction;
    MatrixXd prediction;
    MatrixXd prediction_tmp;
    MatrixXd prediction_vector;
    MatrixXd point_cloud_vector;
    point_cloud_vector.resize(3 * point_cloud.cols(), 1);
    point_cloud_vector = Map<MatrixXd>(point_cloud.data(), point_cloud.size(), 1);
    std::tie(valid_prediction, prediction_vector) = prediction_->predictPointCloud(object_pose_, point_cloud_vector, true);
    prediction = Map<MatrixXd>(prediction_vector.col(0).data(), 3, prediction_vector.col(0).size() / 3);

    // {
    //     save = false;
    //     std::ofstream pc;
    //     pc.open("./pc.OFF");
    //     pc << "OFF" << std::endl;
    //     pc << point_cloud.cols() << " 0 0" << std::endl;
    //     pc << point_cloud.transpose();
    //     pc.close();

    //     std::ofstream proj;
    //     proj.open("./proj.OFF");
    //     proj << "OFF" << std::endl;
    //     proj << prediction.cols() << " 0 0" << std::endl;
    //     proj << prediction.transpose();
    //     proj.close();
    // }

    // Initialize furthest neighbour search
    // KFNModel* kfn = new KFNModel();
    // kfn->TreeType() = KFNModel::KD_TREE;
    // kfn->RandomBasis() = false;

    bool found_outlier = false;
    std::vector<int> excluded_points;

    std::cout << "Number points: " << point_cloud.cols() << std::endl;
    point_cloud_tmp = point_cloud;
    prediction_tmp = prediction;
    do
    {
        if (excluded_points.size() != 0)
        {
            point_cloud_tmp.resize(point_cloud.rows(), point_cloud.cols() - excluded_points.size());
            prediction_tmp.resize(point_cloud_tmp.rows(), point_cloud_tmp.cols());
            for (std::size_t i = 0, j = 0; i < point_cloud.cols(); i++)
            {
                if (std::find(excluded_points.begin(), excluded_points.end(), i) == excluded_points.end())
                {
                    point_cloud_tmp.col(j).swap(point_cloud.col(i));
                    prediction_tmp.col(j).swap(prediction.col(i));
                    j++;
                }
            }
            excluded_points.clear();
        }

        // std::ofstream pc;
        // pc.open("./pc" + std::to_string(counter) + ".OFF");
        // pc << "OFF" << std::endl;
        // pc << point_cloud_tmp.cols() << " 0 0" << std::endl;
        // pc << point_cloud_tmp.transpose();
        // pc.close();

        // Store reference and query sets for furthest neighbour search
        arma::mat point_cloud_query = arma::mat(point_cloud_tmp.data(), point_cloud_tmp.rows(), point_cloud_tmp.cols(), false, false);
        MatrixXd pc_copy = point_cloud_tmp;
        arma::mat point_cloud_reference = arma::mat(pc_copy.data(), pc_copy.rows(), pc_copy.cols(), false, false);
        // kfn->BuildModel(std::move(point_cloud_reference), size_t(20), DUAL_TREE_MODE, 0.0);
        DrusillaSelect<> akfn(point_cloud_reference, 5, 5);

        // Perform fn search
        arma::Mat<size_t> neighbors;
        arma::mat distances;
        // kfn->Search(std::move(point_cloud_query), 1, neighbors, distances);
        akfn.Search(point_cloud_query, 1, neighbors, distances);

        // std::ofstream fn;
        // fn.open("./fn" + std::to_string(counter) + ".OFF");
        // fn << "OFF" << std::endl;
        // fn << point_cloud_tmp.cols() << " 0 0" << std::endl;
        // for (std::size_t i = 0; i < point_cloud_tmp.cols(); i++)
        //     fn << point_cloud_tmp.col(neighbors(i)).transpose() << std::endl;
        // fn.close();

        found_outlier = false;
        for (std::size_t i = 0; i < point_cloud_tmp.cols(); i++)
        {
            // Query point
            Vector3d q = point_cloud_tmp.col(i);

            // Projected point
            Vector3d q_p = prediction_tmp.col(i);

            // Furthest point to q
            Vector3d f = point_cloud_tmp.col(neighbors(i));

            // Projected point
            Vector3d f_p = prediction_tmp.col(neighbors(i));

            // Distance in real point cloud
            double dist_real = (q - f).norm();

            // Distance in projection
            double dist_proj = (q_p - f_p).norm();

            if (abs(dist_proj - dist_real) > 0.01)
            {
                // Distance of outlier candidates to their projections
                double dist_1 = (q - q_p).norm();
                double dist_2 = (f - f_p).norm();
                int excluded;
                if (dist_1 > dist_2)
                    excluded = i;
                else
                    excluded = neighbors(i);
                excluded_points.push_back(excluded);
                found_outlier = true;
                break;
            }
        }

        point_cloud = point_cloud_tmp;
        prediction = prediction_tmp;
    } while(found_outlier);
    std::cout << "Inliers: " << point_cloud.cols() << std::endl;;

    visual_data_size_ = point_cloud.cols();

    // Evaluate centroid of point cloud and discard too far points
    // VectorXd centroid = (point_cloud.topRows<3>().rowwise().sum()) / point_cloud.cols();
    // VectorXi good_points(point_cloud.cols());
    // for (int i = 0 ; i < point_cloud.cols(); i++)
    // {
    //     good_points(i) = 0;
    //     if ((point_cloud.topRows<3>().col(i) - centroid).norm() < visual_outlier_threshold_)
    //         good_points(i) = 1;
    // }
    // visual_data_size_ = good_points.sum();

    // Take only valid visual points
    // MatrixXd points(3, visual_data_size_);
    // for (int i = 0, j = 0; i < point_cloud.cols(); i++)
    // {
    //     if (good_points(i) == 1)
    //     {
    //         point_cloud.topRows<3>().col(i).swap(points.col(j));
    //         j++;
    //     }
    // }

    // Resize measurements to be a column vector.
    measurement_.resize(3 * point_cloud.cols(), 1);
    measurement_.swap(Map<MatrixXd>(point_cloud.data(), point_cloud.size(), 1));
    // measurement_.resize(3 * points.cols(), 1);
    // measurement_.swap(Map<MatrixXd>(points.data(), points.size(), 1));

    // Log measurements
    logger(measurement_.transpose());

    return true;
}


std::pair<std::size_t, std::size_t> ObjectMeasurements::getOutputSize() const
{
    return std::make_pair(measurement_.size(), 0);
}


std::pair<bool, bfl::Data> ObjectMeasurements::predictedMeasure(const Eigen::Ref<const Eigen::MatrixXd>& cur_states) const
{
    bool valid_measure;
    bfl::Data measurement;
    std::tie(valid_measure, measurement) = measure();

    if (!valid_measure)
        return std::make_pair(false, Data());

    bool valid_prediction;
    MatrixXd prediction;
    std::tie(valid_prediction, prediction) = prediction_->predictPointCloud(cur_states, any::any_cast<MatrixXd>(measurement).col(0), false);

    if (!valid_prediction)
        return std::make_pair(false, Data());

    return std::make_pair(true, prediction);
}


std::pair<bool, bfl::Data> ObjectMeasurements::innovation(const bfl::Data& predicted_measurements, const bfl::Data& measurements) const
{
    MatrixXd innovation = -(any::any_cast<MatrixXd>(predicted_measurements).colwise() - any::any_cast<MatrixXd>(measurements).col(0));

    return std::make_pair(true, std::move(innovation));
}


std::pair<bool, Eigen::MatrixXd> ObjectMeasurements::getNoiseCovarianceMatrix() const
{
    std::string err = "ObjectMeasurements::ctor. Error: method not implemented.";
    throw(std::runtime_error(err));
}


std::pair<bool, MatrixXd> ObjectMeasurements::getVisualNoiseCovarianceMatrix() const
{
    return std::make_pair(true, visual_noise_covariance_);
}


std::pair<bool, MatrixXd> ObjectMeasurements::getTactileNoiseCovarianceMatrix() const
{
    if (!has_tactile_noise_covariance_)
        return std::make_pair(false, MatrixXd());

    return std::make_pair(true, tactile_noise_covariance_);
}


std::size_t ObjectMeasurements::getVisualDataSize() const
{
    return visual_data_size_;
}


std::size_t ObjectMeasurements::getTactileDataSize() const
{
    return tactile_data_size_;
}


bool ObjectMeasurements::setProperty(const std::string& property)
{
    if (property == "reset")
    {
        reset();

        return true;
    }
    else if(property == "enable_log")
    {
        if (enable_log_)
            enable_log(log_path_, "object-tracking");
    }
    else if (property == "measurements_available")
    {
        return measurements_available_;
    }

    return false;
}


void ObjectMeasurements::setObjectPose(const Eigen::VectorXd& pose)
{
    object_pose_ = pose;
}


bool ObjectMeasurements::getDepth()
{
    std::string mode = depth_fetch_mode_;
    if (!depth_initialized_)
    {
        // in case a depth was never received
        // it is required to wait at least for the first image in blocking mode
        mode = "new_image";
    }

    bool valid_depth;
    MatrixXf tmp_depth;
    std::tie(valid_depth, tmp_depth) = camera_->getDepthImage(mode == "new_image");

    if (valid_depth)
    {
        depth_ = std::move(tmp_depth);
        depth_initialized_ = true;
    }

    if (mode == "skip")
        return depth_initialized_ && valid_depth;
    else
        return depth_initialized_;
}


void ObjectMeasurements::reset()
{
    depth_initialized_ = false;

    measurements_available_ = true;

    camera_->reset();

    segmentation_->setProperty("reset");

    disable_log();
}

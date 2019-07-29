/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ObjectMeasurements.h>

#include <mlpack/methods/approx_kfn/drusilla_select.hpp>

#include <armadillo>
#include <vector>

using namespace bfl;
using namespace Eigen;
using namespace mlpack::neighbor;
using MatrixXu = Matrix<std::size_t, Dynamic, Dynamic>;

ObjectMeasurements::ObjectMeasurements
(
    std::unique_ptr<Camera> camera,
    std::shared_ptr<PointCloudSegmentation> segmentation,
    std::shared_ptr<PointCloudPrediction> prediction,
    const Eigen::Ref<const Eigen::MatrixXd>& visual_noise_covariance,
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
    const Eigen::Ref<const Eigen::MatrixXd>& visual_noise_covariance,
    const Eigen::Ref<const Eigen::MatrixXd>& tactile_noise_covariance,
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
    std::tie(valid_point_cloud, point_cloud) = segmentation_->extractPointCloud(*camera_, depth_, 10.0);
    if (!valid_point_cloud)
        return false;

    // Peform outlier rejection

    // Predict point cloud using current object pose
    MatrixXd prediction;
    MatrixXd prediction_vector;
    std::tie(std::ignore, prediction_vector) = prediction_->predictPointCloud(object_pose_, Map<MatrixXd>(point_cloud.data(), point_cloud.size(), 1), true);
    prediction = Map<MatrixXd>(prediction_vector.col(0).data(), 3, prediction_vector.col(0).size() / 3);

    // Temporary storage
    MatrixXd point_cloud_tmp;
    MatrixXd prediction_tmp;
    VectorXi excluded_points = VectorXi::Zero(point_cloud.cols());

    point_cloud_tmp.swap(point_cloud);
    prediction_tmp.swap(prediction);
    do
    {
        // Move inliers to temporary storage
        if (excluded_points.sum() != 0)
        {
            point_cloud_tmp.resize(point_cloud.rows(), point_cloud.cols() - excluded_points.sum());
            prediction_tmp.resize(point_cloud_tmp.rows(), point_cloud_tmp.cols());
            for (std::size_t i = 0, j = 0; i < point_cloud.cols(); i++)
            {
                if (excluded_points(i) == 0)
                {
                    point_cloud_tmp.col(j).swap(point_cloud.col(i));
                    prediction_tmp.col(j).swap(prediction.col(i));
                    j++;
                }
            }
            excluded_points = VectorXi::Zero(point_cloud_tmp.cols());
        }

        // Initialize DrusillaSelect
        bool use_drusilla = true;
        arma::mat point_cloud_reference = arma::mat(point_cloud_tmp.data(), point_cloud_tmp.rows(), point_cloud_tmp.cols(), false, false);
        DrusillaSelect<>* akfn;
        try
        {
            std::size_t drusilla_l = 5;
            std::size_t drusilla_m = 5;
            akfn = new DrusillaSelect<>(point_cloud_reference, drusilla_l, drusilla_m);
        }
        catch (std::invalid_argument)
        {
            // If drusilla_l * drusilla_m > point_cloud_tmp.cols()
            // DrusillaSelect cannot be used
            use_drusilla = false;
        }

        // Search for outliers
        std::size_t k = 10;
        for (std::size_t i = 0; i < point_cloud_tmp.cols(); i++)
        {
            // Query point
            Vector3d q = point_cloud_tmp.col(i);

            MatrixXu neighbors;
            if (use_drusilla)
            {
                arma::Mat<size_t> neighbors_arma;
                arma::mat distances;

                arma::mat query_arma(q.data(), 3, 1, false, false);
                akfn->Search(query_arma, k, neighbors_arma, distances);
                neighbors = Map<MatrixXu>(neighbors_arma.memptr(), neighbors_arma.n_rows, neighbors_arma.n_cols);
            }
            else
            {
                // Since DrusillaSelect is not available, a brute force approach with k = 1 is used
                std::size_t max_index;
                (point_cloud_tmp.colwise() - q).colwise().norm().maxCoeff(&max_index);
                neighbors.resize(1, 1);
                neighbors(0, 0) = max_index;
            }

            // Projected point
            Vector3d q_p = prediction_tmp.col(i);

            for (std::size_t j = 0; j < neighbors.rows(); j++)
            {
                // Furthest point to q
                Vector3d f = point_cloud_tmp.col(neighbors.col(0)(j));

                // Projected point
                Vector3d f_p = prediction_tmp.col(neighbors.col(0)(j));

                // Distance in real point cloud
                double dist_real = (q - f).norm();

                // Distance in projection
                double dist_proj = (q_p - f_p).norm();

                if (abs(dist_proj - dist_real) > 0.01)
                {
                    // Distance of outlier candidates to their projections
                    double dist_1 = (q - q_p).norm();
                    double dist_2 = (f - f_p).norm();
                    if (dist_1 > dist_2)
                        excluded_points(i) = 1;
                    else
                        excluded_points(neighbors.col(0)(j))  = 1;
                }
            }
        }

        point_cloud.swap(point_cloud_tmp);
        prediction.swap(prediction_tmp);
        // std::cout << point_cloud.cols() << std::endl;
    } while(excluded_points.sum() != 0);

    // Extract 2d measurements of segmentation contour
    MatrixXd segmentation_contour;
    std::tie(std::ignore, segmentation_contour) = segmentation_->extractSegmentation();

    // Resize measurements to be a column vector.
    visual_data_size_ = point_cloud.cols();
    measurement_.resize(3 * visual_data_size_  + segmentation_contour.cols() * 2, 1);
    measurement_.col(0).head(visual_data_size_ * 3).swap(Map<MatrixXd>(point_cloud.data(), point_cloud.size(), 1));
    measurement_.col(0).tail(segmentation_contour.cols() * 2).swap(Map<MatrixXd>(segmentation_contour.data(), segmentation_contour.cols() * 2, 1));

    // Log measurements
    logger(measurement_.col(0).head(visual_data_size_ * 3).transpose());

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
    MatrixXd prediction_3d;
    std::tie(valid_prediction, prediction_3d) = prediction_->predictPointCloud(cur_states, any::any_cast<MatrixXd>(measurement).col(0).head(visual_data_size_ * 3), false);

    if (!valid_prediction)
        return std::make_pair(false, Data());

    // Predict 2d measurements of segmentation contour
    std::vector<MatrixXd> predicted_segmentation_contours;
    std::tie(std::ignore, predicted_segmentation_contours) = predictSegmentation(cur_states);

    return std::make_pair(true, std::make_tuple(visual_data_size_, prediction_3d, predicted_segmentation_contours));
}


std::pair<bool, std::vector<MatrixXd>> ObjectMeasurements::predictSegmentation(const Ref<const MatrixXd>& object_poses) const
{
    // Get camera pose
    bool valid_pose = false;
    Transform<double, 3, Affine> camera_pose;
    std::tie(valid_pose, camera_pose) = camera_->getCameraPose(false);
    if (!valid_pose)
        return std::make_pair(false, std::vector<MatrixXd>());

    // Get camera intrinsic parameters
    bool valid_camera_parameters;
    CameraParameters camera_parameters;
    std::tie(valid_camera_parameters, camera_parameters) = camera_->getIntrinsicParameters();
    if (!valid_camera_parameters)
        return std::make_pair(false, std::vector<MatrixXd>());

    // Populate segmentation
    std::vector<MatrixXd> segmentation_matrices;
    for (std::size_t i = 0; i < object_poses.cols(); i++)
    {
        Transform<double, 3, Affine> object_pose;
        object_pose = Translation<double, 3>(object_poses.col(i).head<3>());
        AngleAxisd angle_axis(AngleAxisd(object_poses.col(i).tail<3>()(0), Vector3d::UnitZ()) *
                              AngleAxisd(object_poses.col(i).tail<3>()(1), Vector3d::UnitY()) *
                              AngleAxisd(object_poses.col(i).tail<3>()(2), Vector3d::UnitX()));
        object_pose.rotate(angle_axis);
        Transform<double, 3, Affine> cam_to_object = camera_pose.inverse() * object_pose;

        // Get the model virtual point cloud from class PointCloudPrediction
        MatrixXd cloud = prediction_->evaluateModel(cam_to_object);

        // Transform to a cv contour, i.e. a std::vector of std::vector of cv::Point
        // std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Point> contour(cloud.cols());

        for (std::size_t j = 0; j < cloud.cols(); j++)
        {
            int u = camera_parameters.cx + cloud.col(j)(0) / cloud.col(j)(2) * camera_parameters.fx;
            int v = camera_parameters.cy + cloud.col(j)(1) / cloud.col(j)(2) * camera_parameters.fy;

            contour.at(j) = cv::Point(u, v);
        }

        // Render the hull on an image
        std::vector<std::vector<cv::Point>> hull(1);
        cv::convexHull(contour, hull[0]);
        cv::Mat mask(camera_parameters.width, camera_parameters.height, CV_8UC1, cv::Scalar(0));
        cv::drawContours(mask, hull, 0, 255, CV_FILLED);

        // Extract contours from the mask
        std::vector<std::vector<cv::Point>> extracted_contours;
        cv::findContours(mask, extracted_contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        // Store coordinates
        std::size_t number_points = 0;
        for (std::size_t j = 0; j < extracted_contours.size(); j++)
            number_points += extracted_contours.at(j).size();

        std::size_t p = 0;
        MatrixXd segmentation(2, number_points);
        for (std::size_t j = 0; j < extracted_contours.size(); j++)
            for (std::size_t k = 0; k < extracted_contours.at(j).size(); k++)
            {
                segmentation.col(p)(0) = extracted_contours.at(j).at(k).x;
                segmentation.col(p)(1) = extracted_contours.at(j).at(k).y;
                p++;
            }
        segmentation_matrices.push_back(segmentation);
    }

    return std::make_pair(true, segmentation_matrices);
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

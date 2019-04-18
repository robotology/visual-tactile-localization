/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <BoundingBoxSegmentation.h>
#include <CameraParameters.h>

#include <yarp/cv/Cv.h>

using namespace Eigen;
using namespace yarp::sig;
using namespace yarp::cv;


BoundingBoxSegmentation::BoundingBoxSegmentation(const std::string& port_prefix, const Eigen::Ref<const Eigen::VectorXd>& bounding_box, const std::size_t& depth_stride) :
    bounding_box_(bounding_box),
    depth_stride_(depth_stride)
{
    if (!port_image_out_.open("/" + port_prefix + "/segmentation:o"))
    {
        std::string err = log_ID_ + "::ctor. Error: cannot open bounding box output port.";
        throw(std::runtime_error(err));
    }
}


BoundingBoxSegmentation::~BoundingBoxSegmentation()
{
    port_image_out_.close();
}


bool BoundingBoxSegmentation::freezeSegmentation(Camera& camera)
{
    // Get camera pose
    bool valid_pose = false;
    std::tie(valid_pose, camera_pose_) = camera.getCameraPose(true);
    if (!valid_pose)
        return false;

    // Get camera parameters
    bool valid_parameters = false;
    CameraParameters camera_parameters;
    std::tie(valid_parameters, camera_parameters) = camera.getIntrinsicParameters();
    if (!valid_parameters)
        return false;

    // Create white mask using the bounding box
    cv::Mat bounding_box_mask(camera_parameters.height, camera_parameters.width, CV_8UC1, cv::Scalar(0));
    cv::Point tl(int(bounding_box_(0)), int(bounding_box_(1)));
    cv::Point br(int(bounding_box_(2)), int(bounding_box_(3)));
    cv::rectangle(bounding_box_mask, tl, br, cv::Scalar(255), CV_FILLED);

    // Draw the bounding box on the camera image
    drawBoundingBoxOnCamera(bounding_box_mask, camera);

    // Find non zero coordinates
    cv::Mat non_zero_coordinates;
    cv::findNonZero(bounding_box_mask, non_zero_coordinates);

    // Fill coordinates vector
    coordinates_.clear();
    for (std::size_t i = 0; i < non_zero_coordinates.total(); i+= depth_stride_)
    {
        cv::Point& p = non_zero_coordinates.at<cv::Point>(i);
        coordinates_.push_back(std::make_pair(p.x, p.y));
    }

    return true;
}


std::pair<bool, Eigen::MatrixXd> BoundingBoxSegmentation::extractPointCloud(Camera& camera, const Eigen::Ref<const Eigen::MatrixXf>& depth, const double& max_depth)
{
    // Find valid points according to depth
    VectorXi valid_points(coordinates_.size());
    for (std::size_t i = 0; i < valid_points.size(); i++)
    {
        valid_points(i) = 0;

        float depth_u_v = depth(coordinates_[i].second, coordinates_[i].first);

        if ((depth_u_v > 0) && (depth_u_v < max_depth))
            valid_points(i) = 1;
    }

    // Check if there are valid points
    std::size_t num_valids = valid_points.sum();
    if (num_valids == 0)
        return std::make_pair(false, MatrixXd());

    // Get camera parameters
    bool valid_parameters = false;
    CameraParameters camera_parameters;
    std::tie(valid_parameters, camera_parameters) = camera.getIntrinsicParameters();
    if (!valid_parameters)
        return std::make_pair(false, MatrixXd());

    // Get deprojection matrix
    bool valid_deprojection_matrix = false;
    MatrixXd deprojection_matrix;
    std::tie(valid_deprojection_matrix, deprojection_matrix) = camera.getDeprojectionMatrix();
    if (!valid_deprojection_matrix)
        return std::make_pair(false, MatrixXd());

    // Store only valid points
    MatrixXd points(3, num_valids);
    for (int i = 0, j = 0; i < coordinates_.size(); i++)
    {
        if(valid_points(i) == 1)
        {
            const int& u = coordinates_[i].first;
            const int& v = coordinates_[i].second;

            points.col(j) = deprojection_matrix.col(u * camera_parameters.height + v) * depth(v, u);

            j++;
        }
    }

    // Points with respect to robot root frame
    MatrixXd points_robot = camera_pose_ * points.colwise().homogeneous();

    return std::make_pair(true, points);
}


bool BoundingBoxSegmentation::getProperty(const std::string& property) const
{
    return true;
}


bool BoundingBoxSegmentation::setProperty(const std::string& property)
{
    return true;
}


void BoundingBoxSegmentation::drawBoundingBoxOnCamera(const cv::Mat& bounding_box_mask, Camera& camera)
{
    // Get current image
    bool valid_rgb_image = false;
    cv::Mat image_in;
    std::tie(valid_rgb_image, image_in) = camera.getRgbImage(false);

    if (!valid_rgb_image)
        return;

    // Draw bounding box on current image for debugging purposes
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bounding_box_mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    cv::drawContours(image_in, contours, 0, cv::Scalar(0, 255, 0));

    cv::cvtColor(image_in, image_in, cv::COLOR_BGR2RGB);
    ImageOf<PixelRgb>& image_out = port_image_out_.prepare();
    image_out = fromCvMat<PixelRgb>(image_in);
    port_image_out_.write();
}

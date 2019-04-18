/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <MaskSegmentation.h>

#include <yarp/cv/Cv.h>
#include <yarp/os/Value.h>

using namespace Eigen;
using namespace yarp::cv;
using namespace yarp::os;
using namespace yarp::sig;


MaskSegmentation::MaskSegmentation(const std::string& port_prefix, const std::string& mask_name, const std::size_t& depth_stride) :
    mask_name_(mask_name)
{
    if (!port_image_out_.open("/" + port_prefix + "/segmentation:o"))
    {
        std::string err = log_ID_ + "::ctor. Error: cannot open bounding box output port.";
        throw(std::runtime_error(err));
    }

    if (!port_detection_info_in_.open("/" + port_prefix + "/mask_detectionInfo:i"))
    {
        std::string err = log_ID_ + "::ctor. Error: cannot open bounding box output port.";
        throw(std::runtime_error(err));
    }

    if (!(mask_rpc_client_.open("/" + port_prefix + "/mask_rpc:o")))
    {
        std::string err = log_ID_ + "::ctor. Error: cannot open mask rpc client port.";
        throw(std::runtime_error(err));
    }
}


MaskSegmentation::~MaskSegmentation()
{
    port_image_out_.close();

    port_detection_info_in_.close();

    mask_rpc_client_.close();
}


bool MaskSegmentation::freezeSegmentation(Camera& camera)
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

    // Get bounding box
    bool valid_bounding_box = false;
    Vector4d bounding_box;
    std::tie(valid_bounding_box, bounding_box) = getBoundingBox();
    if (!valid_bounding_box)
        return false;

    // Get points along the mask
    bool valid_mask = false;
    std::vector<cv::Point> mask_points;
    std::tie(valid_mask, mask_points) = getMask(bounding_box);
    if (!valid_mask)
        return false;

    // Draw the mask on the camera image
    drawMaskOnCamera(mask_points, camera);

    // Create actual white mask on dark background
    cv::Mat mask(camera_parameters.height, camera_parameters.width, CV_8UC1, cv::Scalar(0));
    std::vector<std::vector<cv::Point>> vector_mask_points;
    vector_mask_points.push_back(mask_points);
    cv::drawContours(mask, vector_mask_points, 0, cv::Scalar(255), CV_FILLED);

    // Find non zero coordinates
    cv::Mat non_zero_coordinates;
    cv::findNonZero(mask, non_zero_coordinates);

    // Fill coordinates vector
    coordinates_.clear();
    for (std::size_t i = 0; i < non_zero_coordinates.total(); i+= depth_stride_)
    {
        cv::Point& p = non_zero_coordinates.at<cv::Point>(i);
        coordinates_.push_back(std::make_pair(p.x, p.y));
    }

    return true;
}


std::pair<bool, MatrixXd> MaskSegmentation::extractPointCloud(Camera& camera, const Ref<const MatrixXf>& depth, const double& max_depth)
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


bool MaskSegmentation::getProperty(const std::string& property) const
{
    return true;
}


bool MaskSegmentation::setProperty(const std::string& property)
{
    return true;
}


std::pair<bool, Vector4d> MaskSegmentation::getBoundingBox()
{
    // need to use mask_name_ here
    Bottle* info = port_detection_info_in_.read(true);

    if ((info == nullptr) || (info->size() == 0))
        return std::make_pair(false, Vector4d());

    Bottle* detections = info->get(0).asList();

    if ((detections == nullptr) || (detections->size() == 0))
        return std::make_pair(false, Vector4d());

    for (std::size_t i = 0; i < detections->size(); i++)
    {
        Bottle* detection = detections->get(i).asList();

        if (detection == nullptr)
            continue;

        std::string detection_name = detection->get(0).asString();

        if (detection_name == mask_name_)
        {
            Bottle* bounding_box_bottle = detection->get(2).asList();
            Vector4d bounding_box;

            bounding_box(0) = bounding_box_bottle->get(0).asInt();
            bounding_box(1) = bounding_box_bottle->get(1).asInt();
            bounding_box(2) = bounding_box_bottle->get(2).asInt();
            bounding_box(3) = bounding_box_bottle->get(3).asInt();

            return std::make_pair(false, bounding_box);
        }
    }

    return std::make_pair(false, Vector4d());
}


std::pair<bool, std::vector<cv::Point>> MaskSegmentation::getMask(const Ref<const VectorXd>& bounding_box)
{
    //  command message format: [get_component_around center_u center_v]
    int center_u = int((bounding_box(0) + bounding_box(2)) / 2.0);
    int center_v = int((bounding_box(1) + bounding_box(3)) / 2.0);

    Bottle cmd, reply;
    cmd.addString("get_component_around");
    cmd.addInt(center_u);
    cmd.addInt(center_v);

    if ((!mask_rpc_client_.write(cmd, reply)) ||
        (reply.size() < 1))
        return std::make_pair(false, std::vector<cv::Point>());

    Bottle* mask = reply.get(0).asList();
    if ((mask == nullptr) || (mask->size() == 0))
        return std::make_pair(false, std::vector<cv::Point>());

    std::vector<cv::Point> points;
    for (std::size_t i = 0; i < mask->size(); i++)
    {
        Bottle* point_bottle = mask->get(i).asList();

        points.push_back(cv::Point(point_bottle->get(0).asInt(), point_bottle->get(1).asInt()));
    }

    return std::make_pair(true, points);
}


void MaskSegmentation::drawMaskOnCamera(const std::vector<cv::Point>& mask_points, Camera& camera)
{
    // Get current image
    bool valid_rgb_image = false;
    cv::Mat image_in;
    std::tie(valid_rgb_image, image_in) = camera.getRgbImage(false);

    if (!valid_rgb_image)
        return;

    // Draw mask contour on current image for debugging purposes
    std::vector<std::vector<cv::Point>> contours;
    contours.push_back(mask_points);
    cv::drawContours(image_in, contours, 0, cv::Scalar(0, 255, 0));

    cv::cvtColor(image_in, image_in, cv::COLOR_BGR2RGB);
    ImageOf<PixelRgb>& image_out = port_image_out_.prepare();
    image_out = fromCvMat<PixelRgb>(image_in);
    port_image_out_.write();
}

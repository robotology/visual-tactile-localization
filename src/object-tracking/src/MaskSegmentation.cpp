/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <MaskSegmentation.h>

#include <iostream>

#include <yarp/cv/Cv.h>
#include <yarp/os/Value.h>


using namespace Eigen;
using namespace yarp::cv;
using namespace yarp::os;
using namespace yarp::sig;


MaskSegmentation::MaskSegmentation(const std::string& port_prefix, const std::string& mask_name, const std::size_t& depth_stride) :
    mask_name_(mask_name),
    depth_stride_(depth_stride)
{
    if (!port_image_out_.open("/" + port_prefix + "/segmentation:o"))
    {
        std::string err = log_ID_ + "::ctor. Error: cannot open bounding box output port.";
        throw(std::runtime_error(err));
    }

    if (!port_image_in_.open("/" + port_prefix + "/mask:i"))
    {
        std::string err = log_ID_ + "::ctor. Error: cannot open mask input port.";
        throw(std::runtime_error(err));
    }

    // if (!port_detection_info_in_.open("/" + port_prefix + "/mask_detectionInfo:i"))
    // {
    //     std::string err = log_ID_ + "::ctor. Error: cannot open bounding box output port.";
    //     throw(std::runtime_error(err));
    // }

    if (!(mask_rpc_client_.open("/" + port_prefix + "/mask_rpc:o")))
    {
        std::string err = log_ID_ + "::ctor. Error: cannot open mask rpc client port.";
        throw(std::runtime_error(err));
    }
}


MaskSegmentation::~MaskSegmentation()
{
    port_image_out_.close();

    port_image_in_.close();

    // port_detection_info_in_.close();

    mask_rpc_client_.close();
}


bool MaskSegmentation::freezeSegmentation(Camera& camera)
{

    if (!mask_streaming_initialized_)
    {
        mask_streaming_initialized_ = enableMaskStreaming();

        return false;
    }

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

    // Get binary mask
    bool valid_mask = false;
    cv::Mat mask;
    std::tie(valid_mask, mask) = getMask();

    if (valid_mask)
    {
        mask_initialized_ = true;

        mask_ = mask.clone();

        // Find non zero coordinates
        cv::Mat non_zero_coordinates;
        cv::findNonZero(mask_, non_zero_coordinates);

        // Fill coordinates vector
        coordinates_.clear();
        for (std::size_t i = 0; i < non_zero_coordinates.total(); i+= depth_stride_)
        {
            cv::Point& p = non_zero_coordinates.at<cv::Point>(i);
            coordinates_.push_back(std::make_pair(p.x, p.y));
        }
    }

    if(mask_initialized_)
    {
        // Draw the mask on the camera image
        drawMaskOnCamera(mask_, camera);

        return true;
    }

    return false;
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

    return std::make_pair(true, points_robot);
}


bool MaskSegmentation::getProperty(const std::string& property) const
{
    return false;
}


bool MaskSegmentation::setProperty(const std::string& property)
{
    if (property == "reset")
    {
        mask_initialized_ = false;

        mask_streaming_initialized_ = false;

        return true;
    }
    
    return false;
}


void MaskSegmentation::setMaskName(const std::string& mask_name)
{
    mask_name_ = mask_name;
}


// std::pair<bool, Vector4d> MaskSegmentation::getBoundingBox()
// {
//     Bottle* detections = port_detection_info_in_.read(false);

//     if((detections == nullptr) && initialized_)
//         return std::make_pair(true, bounding_box_);

//     if ((detections == nullptr) || (detections->size() == 0))
//     {
// 	if (initialized_)
// 	    return std::make_pair(true, bounding_box_);
// 	else
// 	    return std::make_pair(false, Vector4d());
//     }

//     for (std::size_t i = 0; i < detections->size(); i++)
//     {
//         Bottle* detection = detections->get(i).asList();

//         if (detection == nullptr)
//             continue;

//         std::string detection_name = detection->get(0).asString();

//         if (detection_name == mask_name_)
//         {
//             Bottle* bounding_box_bottle = detection->get(2).asList();

//             bounding_box_(0) = bounding_box_bottle->get(0).asInt();
//             bounding_box_(1) = bounding_box_bottle->get(1).asInt();
//             bounding_box_(2) = bounding_box_bottle->get(2).asInt();
//             bounding_box_(3) = bounding_box_bottle->get(3).asInt();

//             initialized_ = true;

//             return std::make_pair(true, bounding_box_);
//         }
//     }

//     if (initialized_)
//         return std::make_pair(true, bounding_box_);
//     else
//         return std::make_pair(false, Vector4d());
// }


bool MaskSegmentation::enableMaskStreaming()
{
    Bottle cmd, reply;
    cmd.addString("set_segmented_object_mask");
    cmd.addString(mask_name_);

    if(mask_rpc_client_.write(cmd, reply))
    {
        std::string reply_str = reply.get(0).asString();
        if (reply_str == "ack")
            return true;
        else
        {
            std::cerr << log_ID_ + "::ctor. Error: cannot start mask stream." << std::endl
                      << "Response from mask was: " << reply_str << std::endl;
        }
    }
    else
    {
        std::cerr << log_ID_ + "::ctor. Error: cannot start mask stream." << std::endl
                  << "Rpc write failed" << std::endl;
    }

    return false;
}


std::pair<bool, cv::Mat> MaskSegmentation::getMask()
{
    ImageOf<PixelMono>* mask_in;
    mask_in = port_image_in_.read(false);

    if (mask_in == nullptr)
        return std::make_pair(false, cv::Mat());

    cv::Mat mask = yarp::cv::toCvMat(*mask_in);

    cv::Mat non_zero_coordinates;
    cv::findNonZero(mask, non_zero_coordinates);

    if (non_zero_coordinates.total() == 0)
        return std::make_pair(false, cv::Mat());

    return std::make_pair(true, mask);
}


void MaskSegmentation::drawMaskOnCamera(const cv::Mat& mask, Camera& camera)
{
    // Get current image
    bool valid_rgb_image = false;
    cv::Mat image_in;
    std::tie(valid_rgb_image, image_in) = camera.getRgbImage(false);

    if (!valid_rgb_image)
        return;

    // Draw mask contour on current image for debugging purposes
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    for (std::size_t i = 0; i < contours.size(); i++)
        cv::drawContours(image_in, contours, i, cv::Scalar(0, 255, 0), 3);

    cv::cvtColor(image_in, image_in, cv::COLOR_BGR2RGB);
    ImageOf<PixelRgb>& image_out = port_image_out_.prepare();
    image_out = fromCvMat<PixelRgb>(image_in);
    port_image_out_.write();
}

/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <CameraParameters.h>
#include <InHandObjectSegmentation.h>

#include <Eigen/Dense>

#include <yarp/eigen/Eigen.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Vocab.h>


using namespace Eigen;
using namespace yarp::eigen;
using namespace yarp::os;


InHandObjectSegmentation::InHandObjectSegmentation
(
    const std::string& port_prefix,
    const std::size_t& depth_stride,
    std::unique_ptr<ObjectRenderer> object_renderer,
    const Ref<const VectorXd>& initial_bounding_box
) :
    InHandObjectSegmentation(port_prefix, depth_stride, std::move(object_renderer))
{
    bounding_box_ = initial_bounding_box_ = tlBrToCenterWidthHeight(initial_bounding_box);
    user_provided_bounding_box_ = true;
    initialized_ = true;
}


InHandObjectSegmentation::InHandObjectSegmentation
(
    const std::string& port_prefix,
    const std::size_t& depth_stride,
    std::unique_ptr<ObjectRenderer> object_renderer,
    const std::string& IOL_object_name,
    const double& IOL_bounding_box_scale
) :
    InHandObjectSegmentation(port_prefix, depth_stride, std::move(object_renderer))
{
    IOL_object_name_ = IOL_object_name;
    IOL_bounding_box_scale_ = IOL_bounding_box_scale;
}


InHandObjectSegmentation::InHandObjectSegmentation
(
    const std::string& port_prefix,
    const std::size_t& depth_stride,
    std::unique_ptr<ObjectRenderer> object_renderer
) :
    object_renderer_(std::move(object_renderer)),
    depth_stride_(depth_stride)
{
    if (!(hand_pose_port_in_.open("/" + port_prefix + "/hand_pose:i")))
    {
        std::string err = log_ID_ + "::ctor. Error: cannot open hand pose input port.";
        throw(std::runtime_error(err));
    }

    if (!(opc_rpc_client_.open("/" + port_prefix + "/opc/rpc:o")))
    {
        std::string err = log_ID_ + "::ctor. Error: cannot open OPC rpc client port.";
        throw(std::runtime_error(err));
    }
}


InHandObjectSegmentation::~InHandObjectSegmentation()
{
    hand_pose_port_in_.close();
    opc_rpc_client_.close();
}


bool InHandObjectSegmentation::freezeSegmentation(Camera& camera)
{
    while(!initialized_)
    {
        // If not initialized with a user provided bounding box
        // try to retrieve the bounding box from IOL
        std::tie(initialized_, bounding_box_) = retrieveIOLBoundingBox();
        std::this_thread::sleep_for(std::chrono::seconds(1));
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

    // Get 2d coordinates
    coordinates_ = getObject2DCoordinates(camera_pose_, camera_parameters);

    return true;
}


std::pair<bool, MatrixXd> InHandObjectSegmentation::extractPointCloud(Camera& camera, const Ref<const MatrixXf>& depth, const double& max_depth)
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


bool InHandObjectSegmentation::getProperty(const std::string& property) const
{
    if (property == "is_occlusion")
        return is_occlusion_;

    return false;
}


bool InHandObjectSegmentation::setProperty(const std::string& property)
{
    bool outcome = PointCloudSegmentation::setProperty(property);

    if (property == "latch_to_hand")
    {
        hand_latch_requests_++;
    }
    else
        outcome = false;

    return outcome;
}


void InHandObjectSegmentation::reset()
{
    PointCloudSegmentation::reset();

    if (user_provided_bounding_box_)
    {
        bounding_box_ = initial_bounding_box_;
    }
    latch_to_hand_ = false;
    projected_bounding_box_initialized_ = false;
    is_occlusion_ = false;
    hand_latch_requests_ = 0;
}


Vector4d InHandObjectSegmentation::tlBrToCenterWidthHeight(const Ref<const VectorXd>& tl_br_bbox)
{
    const Vector2d& top_left = tl_br_bbox.head<2>();
    const Vector2d& bottom_right = tl_br_bbox.tail<2>();

    // Express bounding box using center_u, center_v, width and height
    Vector4d bbox;
    bbox(0) = (top_left(0) + bottom_right(0)) / 2.0;
    bbox(1) = (top_left(1) + bottom_right(1)) / 2.0;
    bbox(2) = bottom_right(0) - top_left(0);
    bbox(3) = bottom_right(1) - top_left(1);

    return bbox;
}


std::pair<bool, Vector4d> InHandObjectSegmentation::retrieveIOLBoundingBox()
{
    /*
     * Get object bounding box from OPC module given object name.
     *
     * Adapted from https://github.com/robotology/point-cloud-read
     */
    Vector4d bbox(4);

    // Command message format is: [ask] (("prop0" "<" <val0>) || ("prop1" ">=" <val1>) ...)
    Bottle cmd, reply;
    cmd.addVocab(Vocab::encode("ask"));
    Bottle &content = cmd.addList().addList();
    content.addString("name");
    content.addString("==");
    content.addString(IOL_object_name_);

    if(!opc_rpc_client_.write(cmd,reply))
        return std::make_pair(false, Vector4d());

    // reply message format: [nack]; [ack] ("id" (<num0> <num1> ...))
    if (reply.size()>1)
    {
        //  verify that first element is "ack"
        if (reply.get(0).asVocab() == Vocab::encode("ack"))
        {
            //  get list of all id's of objects named obj_name
            if (Bottle* id_field = reply.get(1).asList())
            {
                if (Bottle* id_values = id_field->get(1).asList())
                {
                    //  if there are more objects under the same name, pick the first one
                    int id = id_values->get(0).asInt();

                    //  get the actual bounding box
                    //  command message format:  [get] (("id" <num>) (propSet ("prop0" "prop1" ...)))
                    cmd.clear();
                    cmd.addVocab(Vocab::encode("get"));
                    Bottle& content = cmd.addList();
                    Bottle& list_bid = content.addList();
                    list_bid.addString("id");
                    list_bid.addInt(id);
                    Bottle& list_propSet = content.addList();
                    list_propSet.addString("propSet");
                    Bottle& list_items = list_propSet.addList();
                    list_items.addString("position_2d_left");
                    Bottle reply_prop;
                    opc_rpc_client_.write(cmd,reply_prop);

                    //reply message format: [nack]; [ack] (("prop0" <val0>) ("prop1" <val1>) ...)
                    if (reply_prop.get(0).asVocab() == Vocab::encode("ack"))
                    {
                        if (Bottle* prop_field = reply_prop.get(1).asList())
                        {
                            if (Bottle* position_2d_bb = prop_field->find("position_2d_left").asList())
                            {
                                Vector4d tl_br_bbox;

                                //  position_2d_left contains x,y of top left and x,y of bottom right
                                tl_br_bbox(0) = position_2d_bb->get(0).asInt();
                                tl_br_bbox(1) = position_2d_bb->get(1).asInt();
                                tl_br_bbox(2) = position_2d_bb->get(2).asInt();
                                tl_br_bbox(3) = position_2d_bb->get(3).asInt();

                                // initialize center, width and height
                                bbox =  tlBrToCenterWidthHeight(tl_br_bbox);

                                // scale according to a given multiplier
                                bbox.tail<2>() *= IOL_bounding_box_scale_;

                                return std::make_pair(true, bbox);
                            }
                        }
                    }
                }
            }
        }
    }

    return std::make_pair(false, Vector4d());
}


std::pair<bool, Vector4d> InHandObjectSegmentation::projectObjectToBoundingBox(const Transform<double, 3, Affine> object_pose, const Transform<double, 3, Affine> camera_pose)
{
    bool valid_render = false;
    cv::Mat render;
    std::tie(valid_render, render) = object_renderer_->renderObject(object_pose, camera_pose);
    if (!valid_render)
        return std::make_pair(false, Vector4d());

    // Convert to gray scale
    cv::cvtColor(render, render, CV_BGR2GRAY);

    // Evaluate bounding box
    Vector4d bounding_box;
    cv::Mat points;
    cv::findNonZero(render, points);
    cv::Rect rect = cv::boundingRect(points);

    // Form vector containing center, width and height
    bounding_box(0) = rect.x + rect.width / 2.0;
    bounding_box(1) = rect.y + rect.height / 2.0;
    bounding_box(2) = rect.width;
    bounding_box(3) = rect.height;

    return std::make_pair(true, bounding_box);
}


void InHandObjectSegmentation::updateBoundingBox(const Transform<double, 3, Affine> camera_pose)
{
    // Get the hand pose
    yarp::sig::Vector* hand_pose_yarp = hand_pose_port_in_.read(false);
    if (hand_pose_yarp == nullptr)
        return;

    // Evaluate the current rotation of the hand
    VectorXd current_hand_pose = toEigen(*hand_pose_yarp);
    Matrix3d current_hand_rotation = AngleAxisd(current_hand_pose(6), current_hand_pose.segment(3, 3)).toRotationMatrix();

    if (projected_bounding_box_initialized_)
    {
        // Perturb last object pose available
        Transform<double, 3, Affine> perturbed_pose;
        current_object_pose_.translation() += current_hand_pose.segment(0, 3) - hand_pose_.segment(0, 3);
        // The linear part is just the rotation matrix
        current_object_pose_.linear() = current_hand_rotation * rotation_hand_object_;

        // Evaluate current bounding box
        bool valid_bounding_box;
        Vector4d current_projected_bounding_box;
        std::tie(valid_bounding_box, current_projected_bounding_box) = projectObjectToBoundingBox(current_object_pose_, camera_pose);

        // Update if possible
        if (valid_bounding_box)
        {
            bounding_box_ += (current_projected_bounding_box - projected_bounding_box_);

            // Update for next iteration
            projected_bounding_box_ = current_projected_bounding_box;
        }
    }
    else
    {
        if (object_pose_available_)
        {
            // Initialize for the first time the object pose
            // that will be iteratively updated using only the forward kinematics of the hand
            current_object_pose_ = object_pose_;

            // Initialize for the first time the projected bounding box of the object onto the camera plane
            std::tie(projected_bounding_box_initialized_, projected_bounding_box_) = projectObjectToBoundingBox(current_object_pose_, camera_pose);

            // Store the relative orientation between the hand and the object
            rotation_hand_object_ = current_hand_rotation.transpose() * current_object_pose_.rotation();
        }
    }

    // Update for next iteration
    hand_pose_ = current_hand_pose;
}


std::vector<std::pair<int, int>> InHandObjectSegmentation::getObject2DCoordinates(const Transform<double, 3, Affine> camera_pose, const CameraParameters& camera_parameters)
{
    // Update bounding box
    if (hand_latch_requests_ > hand_latch_requests_threshold_)
        updateBoundingBox(camera_pose);

    // Update all the occlusions
    for (auto& occlusion : occlusions_)
        occlusion->findOcclusionArea();

    // Create white mask using the current bounding box
    cv::Mat bounding_box_mask(camera_parameters.height, camera_parameters.width, CV_8UC1, cv::Scalar(0));
    cv::Point tl(int(bounding_box_(0) - bounding_box_(2) / 2.0), int(bounding_box_(1) - bounding_box_(3) / 2.0));
    cv::Point br(int(bounding_box_(0) + bounding_box_(2) / 2.0), int(bounding_box_(1) + bounding_box_(3) / 2.0));
    cv::rectangle(bounding_box_mask, tl, br, cv::Scalar(255), CV_FILLED);

    // Filter mask taking into account occlusions
    is_occlusion_ = false;
    for (auto& occlusion : occlusions_)
    {
        cv::Mat mask;
        bool valid;
        bool is_occlusion_i;
        std::tie(valid, is_occlusion_i, mask) = occlusion->removeOcclusion(bounding_box_mask);

        if (valid)
        {
            bounding_box_mask = mask.clone();
            is_occlusion_ |= is_occlusion_i;
        }
    }

    // Store a copy of obtained region of interest
    // This is required for debugging purposes
    region_of_interest_ = bounding_box_mask.clone();

    // Find non zero coordinates
    cv::Mat non_zero_coordinates;
    cv::findNonZero(bounding_box_mask, non_zero_coordinates);

    // Fill coordinates vector
    std::vector<std::pair<int, int>> coordinates;
    for (std::size_t i = 0; i < non_zero_coordinates.total(); i+= depth_stride_)
    {
        cv::Point& p = non_zero_coordinates.at<cv::Point>(i);
        coordinates.push_back(std::make_pair(p.x, p.y));
    }

    return coordinates;
}

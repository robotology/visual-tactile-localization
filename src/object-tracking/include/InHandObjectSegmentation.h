/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef INHANDOBJECTSEGMENTATION_H
#define INHANDOBJECTSEGMENTATION_H

#include <Eigen/Dense>

#include <CameraParameters.h>
#include <ObjectRenderer.h>
#include <PointCloudSegmentation.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>

#include <memory>
#include <string>


class InHandObjectSegmentation : public PointCloudSegmentation
{
public:

    InHandObjectSegmentation(const std::string& port_prefix, const std::size_t& depth_stride, std::unique_ptr<ObjectRenderer> object_renderer, const Eigen::Ref<const Eigen::VectorXd>& initial_bounding_box);

    InHandObjectSegmentation(const std::string& port_prefix, const std::size_t& depth_stride, std::unique_ptr<ObjectRenderer> object_renderer, const std::string& IOL_object_name, const double& IOL_bounding_box_scale);

    ~InHandObjectSegmentation();

    bool freezeSegmentation(Camera& camera) override;

    std::pair<bool, Eigen::MatrixXd> extractPointCloud(Camera& camera, const Eigen::Ref<const Eigen::MatrixXf>& depth, const double& max_depth = 1.0) override;

    bool getProperty(const std::string& property) const override;

    bool setProperty(const std::string& property) override;

    void reset() override;

protected:

    InHandObjectSegmentation(const std::string& port_prefix, const std::size_t& depth_stride, std::unique_ptr<ObjectRenderer> object_renderer);

    Eigen::Vector4d tlBrToCenterWidthHeight(const Eigen::Ref<const Eigen::VectorXd>& tl_br_bounding_box);

    std::pair<bool, Eigen::Vector4d> retrieveIOLBoundingBox();

    std::pair<bool, Eigen::Vector4d> projectObjectToBoundingBox(const Eigen::Transform<double, 3, Eigen::Affine> object_pose, const Eigen::Transform<double, 3, Eigen::Affine> camera_pose);

    void updateBoundingBox(const Eigen::Transform<double, 3, Eigen::Affine> camera_pose);

    std::vector<std::pair<int, int>> getObject2DCoordinates(const Eigen::Transform<double, 3, Eigen::Affine> camera_pose, const CameraParameters& camera_parameters);

    void drawROIsOnCamera(Camera& camera);

    /**
     * Object renderer.
     */

    std::unique_ptr<ObjectRenderer> object_renderer_;

    /**
     * Camera pose.
     */
    Eigen::Transform<double, 3, Eigen::Affine> camera_pose_;

    /**
     * Bounding box.
     */

    bool initialized_ = false;

    bool projected_bounding_box_initialized_ = false;

    bool user_provided_bounding_box_ = false;

    Eigen::Vector4d initial_bounding_box_;

    Eigen::Vector4d bounding_box_;

    Eigen::Vector4d projected_bounding_box_;

    /**
     * Region of interest_;
     */

    cv::Mat region_of_interest_;

    std::vector<std::pair<int, int>> coordinates_;

    /**
     * Perturbed object pose.
     */
    Eigen::Transform<double, 3, Eigen::Affine> current_object_pose_;

    /**
     * Hand pose.
     */

    Eigen::VectorXd hand_pose_;

    Eigen::Matrix3d rotation_hand_object_;

    yarp::os::BufferedPort<yarp::sig::Vector> hand_pose_port_in_;

    yarp::os::Stamp hand_pose_stamp_;

    bool latch_to_hand_ = false;

    int hand_latch_requests_ = 0;

    int hand_latch_requests_threshold_ = 10;

    /**
     * Occlusions.
     */
    bool is_occlusion_ = false;

    /**
     * IOL.
     */
    std::string IOL_object_name_;

    double IOL_bounding_box_scale_;

    /**
     * RPC clients.
     */
    yarp::os::RpcClient opc_rpc_client_;

    /**
     * Image showing object and hand considered ROIs
     */
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> port_image_out_;

    const std::string log_ID_ = "InHandObjectSegmentation";
};

#endif



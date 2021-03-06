/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef BOUNDINGBOXESTIMATOR_H
#define BOUNDINGBOXESTIMATOR_H

#include <BayesFilters/GaussianMixture.h>
#include <BayesFilters/EstimatesExtraction.h>

#include <Eigen/Dense>

#include <GazeController.h>

#include <opencv2/opencv.hpp>

#include <SuperimposeMesh/SICAD.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>


class BoundingBoxEstimator
{
public:
    using BBox = std::pair<std::pair<int, int>, std::pair<int, int>>;

    BoundingBoxEstimator
    (
        const BBox initial_bbox,
        const std::size_t number_components,
        const std::string port_prefix,
        const std::string eye_name,
        const std::string obj_mesh_file,
        const std::string sicad_shader_path,
        const std::string IOL_object_name,
        const double IOL_bbox_scale,
        const bool send_mask,
        const Eigen::Ref<const Eigen::MatrixXd>& initial_covariance,
        const Eigen::Ref<const Eigen::MatrixXd>& process_noise_covariance,
        const Eigen::Ref<const Eigen::MatrixXd>& measurement_noise_covariance
    );

    BoundingBoxEstimator
    (
        const std::size_t number_components,
        const std::string port_prefix,
        const std::string eye_name,
        const std::string obj_mesh_file,
        const std::string sicad_shader_path,
        const std::string IOL_object_name,
        const double IOL_bbox_scale,
        const bool send_mask,
        const bool bounding_box_from_port,
        const Eigen::Ref<const Eigen::MatrixXd>& initial_covariance,
        const Eigen::Ref<const Eigen::MatrixXd>& process_noise_covariance,
        const Eigen::Ref<const Eigen::MatrixXd>& measurement_noise_covariance
    );

    ~BoundingBoxEstimator();

    /**
     * Step the estimator.
     */
    void step();

    /**
     * Get the current estimate
     */
    Eigen::VectorXd getEstimate();
    Eigen::VectorXd getEstimate(const Eigen::Ref<const Eigen::VectorXd>& weights);

    /**
     * Reset the estimator
     */
    void reset();

    /**
     * Set the current object 3D estimate to be used as an hint by the bounding box estimator.
     */
    void setObjectPose(const Eigen::Ref<const Eigen::MatrixXd>& pose);

    /**
     * Set the current object 3D estimate to be used as an hint by the bounding box estimator.
     */
    void setObjectPose(const Eigen::Ref<const Eigen::MatrixXd>& pose, const Eigen::Ref<const Eigen::VectorXd>& weights);

    /**
     * Get the number of components.
     */
    std::size_t getNumberComponents();

    /**
     * Enable/disable feedforward term obtained using finite difference of the hand 3D position projected onto the camera plane.
     */
    void enableHandFeedforward(const bool& enable);

    /**
     * Resample particles.
     */
    void resampleParticles(const Eigen::VectorXi& parents);

protected:
    /**
     * Perform prediction.
     */
    void predict();

    /**
     * Perform correction.
     */
    void correct();

    /**
     * Evaluate exogenous input.
     */
    /* Eigen::MatrixXd evalExogenousInput(); */

    /**
     * Evaluate exogenous input.
     */
    Eigen::MatrixXd evalHandExogenousInput();

    /**
     * Retrieve the object bounding box according to iCub OPC (objects property collector).
     * Return a boolean indicating the outcome and a 4-vector containing center, width and height.
     */
    std::pair<bool, Eigen::VectorXd> measure();

    /**
     * Draw the current object mask on the camera image and send the rendered image on the network.
     */
    void sendObjectMask();

    /**
     * Evaluate the new bounding box according to the pose of the object.
     */
    std::pair<bool, Eigen::MatrixXd> updateObjectBoundingBox();

    /**
     * Number of particles used.
     */
    std::size_t number_components_;

    /**
     * Gaussian belief (predicted and corrected).
     */
    bfl::GaussianMixture pred_bbox_;
    bfl::GaussianMixture corr_bbox_;
    bool is_initialized_;
    bool is_hand_exogenous_initialized_;

    /**
     * Noise covariances (process and measurement noise)
     */
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;

    /**
     * User provided initial condition.
     */
    Eigen::VectorXd mean_0_;
    Eigen::MatrixXd cov_0_;
    bool user_provided_mean_0_;

    /**
     * IOL initial condition;
     */
    Eigen::VectorXd iol_mean_0_;

    /**
     * Enable/disable object feedforward term.
     */
    bool enable_hand_feedforward_;
    bool enable_hand_feedforward_first_time_;
    int enable_hand_feedforward_counter_;

    /**
     * Estimates extraction.
     */
    bfl::EstimatesExtraction extractor_;

    /**
     * Interface to iCub cameras.
     */
    GazeController gaze_;

    /**
     * iCub camera selection, width/height in pixels, intrinsic parameters.
     */
    const std::string eye_name_;
    const int cam_width_ = 320;
    const int cam_height_ = 240;
    double cam_fx_;
    double cam_fy_;
    double cam_cx_;
    double cam_cy_;

    /**
     * Instance of superimpose cad
     */
    std::unique_ptr<SICAD> object_sicad_;

    /**
     * Object mask.
     */
    cv::Mat object_mask_;

    /**
     * Object projected bounding box (center, width, height)
     */
    Eigen::MatrixXd proj_bbox_;

    /*
     * Object 3D pose.
     */
    Eigen::MatrixXd object_3d_pose_;
    Eigen::VectorXd object_3d_pose_perturbed_;
    Eigen::VectorXd object_3d_pose_weights_;
    bool is_object_pose_initialized_;

    /*
     * Hand 3D pose.
     */
    Eigen::VectorXd hand_pose_;

    /*
     * Relative rotation between hand and object.
     */
    Eigen::Matrix3d relative_hand_object_rotation_;

    /**
     * IOL object category name (required to initialize the bounding box of the object).
     */
    std::string IOL_object_name_;

    /**
     * Scale factor used to enlarge/reduce the area of the bounding box provided by IOL.
     */
    double IOL_bbox_scale_;

    /**
     * Input port for hand 3D pose required to evaluate the hand feedforward term.
     */
    yarp::os::BufferedPort<yarp::sig::Vector> hand_pose_port_in_;
    yarp::os::Stamp hand_pose_stamp_;

    /**
     * IOL bounding box input/output port.
     * The initial bounding box received by IOL is sent and can be received for debugging purposes.
     */
    yarp::os::BufferedPort<yarp::sig::Vector> iol_bbox_port_in_;

    yarp::os::BufferedPort<yarp::sig::Vector> iol_bbox_port_out_;

    bool bounding_box_from_port_;

    /**
     * RPC clients.
     */
    yarp::os::RpcClient opc_rpc_client_;

    /**
     * Image input/output.
     */
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> port_image_in_;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> port_mask_image_out_;
    bool send_mask_ = false;

    const std::string log_ID_ = "[BOUNDINGBOXESTIMATOR]";
};

#endif

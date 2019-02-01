#ifndef BOUNDINGBOXESTIMATOR_H
#define BOUNDINGBOXESTIMATOR_H

#include <BayesFilters/Gaussian.h>

#include <Eigen/Dense>

#include <GazeController.h>

#include <opencv2/opencv.hpp>

#include <SuperimposeMesh/SICAD.h>

#include <yarp/sig/Image.h>
#include <yarp/os/RpcClient.h>


class BoundingBoxEstimator
{
public:
    using BBox = std::pair<std::pair<int, int>, std::pair<int, int>>;

    BoundingBoxEstimator
    (
        const BBox initial_bbox,
        const std::string port_prefix,
        const std::string eye_name,
        const std::string obj_mesh_file,
        const std::string sicad_shader_path,
        const std::string IOL_object_name,
        const bool send_mask,
        const Eigen::Ref<const Eigen::MatrixXd>& initial_covariance,
        const Eigen::Ref<const Eigen::MatrixXd>& process_noise_covariance,
        const Eigen::Ref<const Eigen::MatrixXd>& measurement_noise_covariance
    );

    BoundingBoxEstimator
    (
        const std::string port_prefix,
        const std::string eye_name,
        const std::string obj_mesh_file,
        const std::string sicad_shader_path,
        const std::string IOL_object_name,
        const bool send_mask,
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

    /**
     * Reset the estimator
     */
    void reset();

    /**
     * Set the current object 3D estimate to be used as an hint by the bounding box estimator.
     */
    void setObjectPose(const Eigen::Ref<const Eigen::VectorXd>& pose);

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
    Eigen::VectorXd evalExogenousInput();

    /**
     * Retrieve the object bounding box according to iCub OPC (objects property collector).
     * Return a boolean indicating the outcome and a 4-vector containing center, width and height.
     */
    std::pair<bool, Eigen::VectorXd> measure();

    /**
     * Draw the current object mask on the camera image and send the rendered image on the network.
     */
    void sendObjectMask(yarp::sig::ImageOf<yarp::sig::PixelRgb>& camera_image);

    /**
     * Update the object mask by projecting the object mesh, according to the last estimate available on the camera plane.
     */
    bool updateObjectMask();

    /**
     * Gaussian belief (predicted and corrected).
     */
    bfl::Gaussian pred_bbox_;
    bfl::Gaussian corr_bbox_;
    bool is_initialized_;
    bool is_exogenous_initialized_;

    std::size_t steady_state_counter_;
    std::size_t steady_state_threshold_;

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
    Eigen::VectorXd proj_bbox_;
    bool is_proj_bbox_initialized_;

    /**
     * Width and height ratio between initial bounding box and predicted bounding box
     * (required to take into account bad scaling of the point cloud).
     */
    double bbox_width_ratio_;
    double bbox_height_ratio_;

    /*
     * Object 3D pose.
     */
    Eigen::VectorXd object_3d_pose_;
    bool is_object_pose_initialized_;

    /**
     * IOL object category name (required to initialize the bounding box of the object).
     */
    std::string IOL_object_name_;

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

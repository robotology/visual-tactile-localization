#ifndef ICUBPOINTCLOUD_H
#define ICUBPOINTCLOUD_H

#include <BayesFilters/Data.h>

#include <Eigen/Dense>

#include <GazeController.h>

#include <PointCloudModel.h>
#include <PointCloudPrediction.h>

#include <SFM.h>

#include <SuperimposeMesh/SICAD.h>

#include <string>
#include <memory>


class iCubPointCloudExogenousData;


class iCubPointCloud : public PointCloudModel
{
public:
    iCubPointCloud
        (
            const string port_prefix,
            const string SFM_context_name,
            const string SFM_config_name,
            const string IOL_object_name,
            const string obj_mesh_file,
            const string sicad_shader_path,
            const string eye_name,
            std::unique_ptr<PointCloudPrediction> prediction,
            const double point_cloud_outlier_threshold,
            const Eigen::Ref<const Eigen::Matrix3d>& noise_covariance_matrix,
            std::shared_ptr<iCubPointCloudExogenousData> exogenous_data,
            const bool send_bounding_box,
            const bool send_mask
        );

        iCubPointCloud
        (
            const string port_prefix,
            const string SFM_context_name,
            const string SFM_config_name,
            const string obj_mesh_file,
            const string sicad_shader_path,
            const string eye_name,
            const std::pair<std::pair<int, int>, std::pair<int, int>> initial_bbox,
            std::unique_ptr<PointCloudPrediction> prediction,
            const double point_cloud_outlier_threshold,
            const Eigen::Ref<const Eigen::Matrix3d>& noise_covariance_matrix,
            std::shared_ptr<iCubPointCloudExogenousData> exogenous_data,
            const bool send_bounding_box,
            const bool send_mask
        );

    virtual ~iCubPointCloud();

    std::pair<bool, bfl::Data> measure() const override;

    bool freezeMeasurements() override;

    std::pair<std::size_t, std::size_t> getOutputSize() const override;

    bool setProperty(const std::string& property) override;

protected:
    iCubPointCloud
    (
        const string port_prefix,
        const string SFM_context_name,
        const string SFM_config_name,
        const string obj_mesh_file,
        const string sicad_shader_path,
        const string eye_name,
        std::unique_ptr<PointCloudPrediction> prediction,
        const double point_cloud_outlier_threshold,
        const Eigen::Ref<const Eigen::Matrix3d>& noise_covariance_matrix,
        std::shared_ptr<iCubPointCloudExogenousData> exogenous_data,
        const bool send_bounding_box,
        const bool send_mask
     );

    /**
     * Retrieve the object bounding box according to iCub OPC (objects property collector).
     * Return a boolean indicating the outcome and a 4-vector containing center, width and height.
     */
    std::tuple<bool, Eigen::VectorXd> retrieveObjectBoundingBox(const string obj_name);

    /**
     * Draw the current bounding box on the camera image and send the rendered image on the network.
     */
    void sendObjectBoundingBox(yarp::sig::ImageOf<PixelRgb>& camera_image);

    /**
     * Draw the current object mask on the camera image and send the rendered image on the network.
     */
    void sendObjectMask(yarp::sig::ImageOf<PixelRgb>& camera_image);

    /**
     * Evaluate the 2D coordinates of the object.
     */
    std::pair<bool, std::vector<std::pair<int, int>>> getObject2DCoordinates(std::size_t stride_u, std::size_t stride_v);

    /**
     * Update the object mask by projecting the object mesh, according to the last estimate available,
     * on the camera plane.
     */
    bool updateObjectMask();

    /**
     * Update the object bounding box using the last object mask available.
     */
    void updateObjectBoundingBox();

    /**
     * Reset the measurement model class.
     */
    void reset();

    /**
     * This is iCub SFM.
     */
    SFM sfm_;

    /**
     * Object bounding box (top-left, bottom-right) of the target object.
     */
    Eigen::VectorXd obj_bbox_;
    Eigen::VectorXd obj_bbox_0_;
    bool use_initial_bbox_;
    bool obj_bbox_set_;

    /**
     * Object projected bounding box (center, width, height)
     */
    Eigen::VectorXd proj_bbox_;

    /**
     * Width and height ratio between initial bounding box and predicted bounding box
     * (required to take into account bad scaling of the point cloud).
     */
    double bbox_width_ratio_;
    double bbox_height_ratio_;

    /**
     * Object mask.
     */
    cv::Mat object_mask_;

    /**
     * IOL object category name (required to initialize the bounding box of the object).
     */
    std::string IOL_object_name_;

    /**
     * RPC clients.
     */
    RpcClient opc_rpc_client_;

    /**
     * Local copy of measurements.
     * A vector of size 3 * L with L the number of points in the point cloud.
     */
    Eigen::MatrixXd measurement_;

    /**
     * Interface to iCub cameras, required to predict the 2D object bounding box.
     */
    GazeController gaze_;

    /**
     * iCub camera selection, width/height in pixels.
     */
    const std::string eye_name_;
    const int cam_width_ = 320;
    const int cam_height_ = 240;

    /**
     * Image input/output.
     */
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> port_image_in_;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> port_bbox_image_out_;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> port_mask_image_out_;
    bool send_bbox_;
    bool send_mask_;

    /**
     * Instance of superimpose cad
     */
    std::unique_ptr<SICAD> object_sicad_;

    /**
     * Point cloud outlier rejection threshold.
     */
    double pc_outlier_threshold_;

    /**
     * Exogenous data used by this class.
     */
    std::shared_ptr<iCubPointCloudExogenousData> exogenous_data_;

    const std::size_t steady_state_thr_ = 100;

    std::size_t steady_state_counter_;
};

class iCubPointCloudExogenousData
{
public:
    iCubPointCloudExogenousData();

    virtual ~iCubPointCloudExogenousData();

    /**
     * Set the object pose.
     */
    virtual void setObjectEstimate(const Eigen::Ref<const Eigen::VectorXd>& pose);

    /**
     * Get the object pose.
     */
    std::pair<bool, Eigen::VectorXd> getObjectEstimate();

    void reset();

protected:
    /**
     * Last object estimate, to be used as hint to evaluate a better bounding box.
     * The vector contains 3 Cartesian coordinates (m), 3 Cartesian velocites (m/s),
     * 3 Euler angle rates (rad/s) and a ZYX Euler angles parametrization (rad).
     */
    Eigen::VectorXd last_estimate_;
    bool obj_estimate_set_;
};

#endif /* ICUBPOINTCLOUD_H */

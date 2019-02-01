#ifndef ICUBPOINTCLOUD_H
#define ICUBPOINTCLOUD_H

#include <BayesFilters/Data.h>

#include <Eigen/Dense>

#include <GazeController.h>
#include <ObjectOcclusion.h>
#include <PointCloudModel.h>
#include <PointCloudPrediction.h>

#include <yarp/sig/Image.h>

#include <string>
#include <memory>


class iCubPointCloudExogenousData;


class iCubPointCloud : public PointCloudModel
{
public:
    iCubPointCloud
    (
        std::unique_ptr<PointCloudPrediction> prediction,
        const Eigen::Ref<const Eigen::Matrix3d>& noise_covariance_matrix,
        const std::string port_prefix,
        const std::string eye_name,
        const std::string depth_fetch_mode,
        const double point_cloud_outlier_threshold,
        const std::size_t point_cloud_u_stride,
        const std::size_t point_cloud_v_stride,
        const bool send_hull,
        std::shared_ptr<iCubPointCloudExogenousData> exogenous_data
    );

    virtual ~iCubPointCloud();

    std::pair<bool, bfl::Data> measure() const override;

    bool freezeMeasurements() override;

    std::pair<std::size_t, std::size_t> getOutputSize() const override;

    bool setProperty(const std::string& property) override;

    void addObjectOcclusion(std::unique_ptr<ObjectOcclusion> object_occlusion);

protected:
    /**
     * Evaluate the 2D coordinates of the object.
     */
    std::vector<std::pair<int, int>> getObject2DCoordinates(const Eigen::Ref<const Eigen::VectorXd>& bounding_box, std::size_t stride_u, std::size_t stride_v);

    /**
     * Cache a default deprojection matrix D of the form:
     * D.col(u * cam_width + v) = [(u - cam_cx)/fx, (v - cam_cy)/fy, 1.0]^{T}
     * in the variable default_deprojection_matrix_
     */
    void setDefaultDeprojectionMatrix();

    /**
     * Retrieve the depth image from the robot.
     */
    bool getDepth();

    /**
     * Evaluate the point cloud starting from the depth image.
     */
    std::tuple<bool, Eigen::MatrixXd, Eigen::VectorXi> get3DPoints(std::vector<std::pair<int, int>>& coordinates_2d, const float z_threshold = 1.0);

    /**
     * Draw the convex hull given the 2d coordinates that are used to extract the point cloud.
     */
    void sendObjectConvexHull(yarp::sig::ImageOf<yarp::sig::PixelRgb>& camera_image, const std::vector<std::pair<int, int>>& coordinates);

    /**
     * Default deprojection matrix.
     */
    Eigen::MatrixXd default_deprojection_matrix_;

    /**
     * Local copy of depht image.
     */
    yarp::sig::ImageOf<yarp::sig::PixelFloat> depth_image_;
    bool depth_initialized_ = false;
    std::string depth_fetch_mode_;

    /**
     * Local copy of measurements.
     * A vector of size 3 * L with L the number of points in the point cloud.
     */
    Eigen::MatrixXd measurement_;

    /**
     * Point cloud outlier rejection threshold.
     */
    double pc_outlier_threshold_;

    /**
     * 2D strides required to subsample the 2d coordinates on the image plane.
     */
    std::size_t pc_u_stride_;
    std::size_t pc_v_stride_;

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
     * Object occlusions.
     */
    std::vector<std::unique_ptr<ObjectOcclusion>> occlusions_;

    /**
     * Image input/output.
     */
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat>> port_depth_in_;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> port_image_in_;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> port_hull_image_out_;
    bool send_hull_;

    /**
     * Exogenous data used by this class.
     */
    std::shared_ptr<iCubPointCloudExogenousData> exogenous_data_;
};

class iCubPointCloudExogenousData
{
public:
    iCubPointCloudExogenousData();

    virtual ~iCubPointCloudExogenousData();

    /**
     * Set the bounding box.
     */
    virtual void setBoundingBox(const Eigen::Ref<const Eigen::VectorXd>& bounding_box);

    /**
     * Get the bounding box.
     */
    std::pair<bool, Eigen::VectorXd> getBoundingBox();

    void reset();

protected:
    Eigen::VectorXd bbox_;

    bool bbox_set_;
};

#endif /* ICUBPOINTCLOUD_H */

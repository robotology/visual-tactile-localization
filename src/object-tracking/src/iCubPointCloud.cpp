#include <iCubPointCloud.h>

#include <yarp/cv/Cv.h>
#include <yarp/eigen/Eigen.h>
#include <yarp/sig/Vector.h>

#include <BayesFilters/Data.h>

#include <Eigen/Dense>

#include <SuperimposeMesh/Superimpose.h>
#include <SuperimposeMesh/SICAD.h>

using namespace bfl;
using namespace yarp::eigen;
using namespace yarp::sig;
using namespace Eigen;


iCubPointCloud::iCubPointCloud
(
    std::unique_ptr<PointCloudPrediction> prediction,
    const Eigen::Ref<const Eigen::Matrix3d>& noise_covariance_matrix,
    const Eigen::Ref<const Eigen::Matrix3d>& tactile_noise_covariance_matrix,
    const std::string port_prefix,
    const std::string eye_name,
    const std::string depth_fetch_mode,
    const double point_cloud_outlier_threshold,
    const std::size_t point_cloud_u_stride,
    const std::size_t point_cloud_v_stride,
    const bool send_hull,
    std::shared_ptr<iCubPointCloudExogenousData> exogenous_data
) :
    PointCloudModel(std::move(prediction), noise_covariance_matrix, tactile_noise_covariance_matrix),
    eye_name_(eye_name),
    depth_fetch_mode_(depth_fetch_mode),
    pc_outlier_threshold_(point_cloud_outlier_threshold),
    pc_u_stride_(point_cloud_u_stride),
    pc_v_stride_(point_cloud_v_stride),
    send_hull_(send_hull),
    exogenous_data_(exogenous_data),
    gaze_(port_prefix)
{
    // Open ports.

    if (!(port_depth_in_.open("/" + port_prefix + "/depth:i")))
    {
        std::string err = "ICUBPOINTCLOUD::CTOR::ERROR\n\tError: cannot open depth input port.";
        throw(std::runtime_error(err));
    }

    if (send_hull_)
    {
        // Open camera input port.
        if(!port_image_in_.open("/" + port_prefix + "/cam/" + eye_name + ":i"))
        {
            std::string err = "VIEWER::CTOR::ERROR\n\tError: cannot open " + eye_name + " camera input port.";
            throw(std::runtime_error(err));
        }

        // Open image output port.
        if(!port_hull_image_out_.open("/" + port_prefix + "/hull:o"))
        {
            std::string err = "VIEWER::CTOR::ERROR\n\tError: cannot open hull output port.";
            throw(std::runtime_error(err));
        }
    }

    // Get iCub cameras intrinsics parameters
    if (!gaze_.getCameraIntrinsics(eye_name, cam_fx_, cam_fy_, cam_cx_, cam_cy_))
    {
        std::string err = "ICUBPOINTCLOUD::CTOR::ERROR\n\tError: cannot retrieve iCub camera intrinsicse.";
        throw(std::runtime_error(err));
    }

    // Configure the default deprojection matrix
    setDefaultDeprojectionMatrix();
}


iCubPointCloud::~iCubPointCloud()
{
    // Close ports
    port_depth_in_.close();

    if (send_hull_)
    {
        port_image_in_.close();
        port_hull_image_out_.close();
    }
}


std::pair<bool, Data> iCubPointCloud::measure() const
{
    return std::make_pair(true, measurement_);
}


bool iCubPointCloud::freezeMeasurements()
{
    // Get bounding box
    bool valid_bbox;
    VectorXd bbox;
    std::tie(valid_bbox, bbox) = exogenous_data_->getBoundingBox();
    if (!valid_bbox)
        return false;

    // Update all the occlusions
    for (auto& occlusion : occlusions_)
        occlusion->findOcclusionArea();

    // Get 2d coordinates
    std::vector<std::pair<int, int>> coordinates;
    coordinates = getObject2DCoordinates(bbox, pc_u_stride_, pc_v_stride_);

    // Send hull over the network
    if (send_hull_)
    {
        // Get input image
        ImageOf<PixelRgb>* image_in;
        image_in = port_image_in_.read(false);

        if (image_in != nullptr)
        {
            // Prepare output image
            ImageOf<PixelRgb>& image_out = port_hull_image_out_.prepare();

            // Copy input to output and wrap around a cv::Mat
            image_out = *image_in;
            cv::Mat image = yarp::cv::toCvMat(image_out);
            cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

            // Draw hulls due to occlusions
            for (auto& occlusion : occlusions_)
                occlusion->drawOcclusionArea(image);

            // Draw object ROI
            drawObjectROI(image);

            // Send the image
            port_hull_image_out_.write();
        }
    }

    // Get depth image
    if(!getDepth())
        return false;

    // Get 3D point cloud.
    bool blocking_call = false;
    bool valid_point_cloud;
    MatrixXd point_cloud;
    std::tie(valid_point_cloud, point_cloud, std::ignore) = get3DPoints(coordinates);
    if (!valid_point_cloud)
        return false;

    // Evaluate centroid of point cloud
    VectorXd centroid = (point_cloud.rowwise().sum()) / point_cloud.cols();
    VectorXi good_points(point_cloud.cols());

    for (int i = 0 ; i < point_cloud.cols(); i++)
    {
        good_points(i) = 0;
        if ((point_cloud.col(i) - centroid).norm() < pc_outlier_threshold_)
            good_points(i) = 1;
    }


    // Check if contact is occurring
    VectorXd tactile_points;
    int number_tactile_points = 0;
    if ((contacts_ != nullptr) && (contacts_->freezeMeasurements()))
    {
        tactile_points = contacts_->measure();

        number_tactile_points = tactile_points.size() / 3;
    }

    // Allocate storage
    MatrixXd points(3, good_points.sum() + number_tactile_points);

    // Take only valid visual points
    int j = 0;
    for (int i = 0; i < point_cloud.cols(); i++)
    {
        if (good_points(i) == 1)
        {
            point_cloud.col(i).swap(points.col(j));
            j++;
        }
    }

    // If any, take also contact points
    for (int i = 0; i < number_tactile_points; i++)
    {
        tactile_points.segment(i * 3, 3).swap(points.col(j));

	j++;
    }

    // Set the size of the visual part of the point cloud
    visual_point_cloud_size_ = good_points.sum();

    // Resize measurements to be a column vector.
    measurement_.resize(3 * points.cols(), 1);
    measurement_.swap(Map<MatrixXd>(points.data(), points.size(), 1));

    logger(measurement_.transpose());

    return true;
}


std::pair<std::size_t, std::size_t> iCubPointCloud::getOutputSize() const
{
    return std::make_pair(measurement_.size(), 0);
}


bool iCubPointCloud::setProperty(const std::string& property)
{
    bool ok = false;

    if (property == "reset")
    {
        // do reset
        ok = true;
    }

    return ok;
}


void iCubPointCloud::addObjectOcclusion(std::unique_ptr<ObjectOcclusion> object_occlusion)
{
    occlusions_.push_back(std::move(object_occlusion));
}


void iCubPointCloud::addObjectContacts(std::unique_ptr<iCubHandContactsModel> object_contacts)
{
    contacts_ = std::move(object_contacts);
}


int iCubPointCloud::getVisualPointCloudSize()
{
    return visual_point_cloud_size_;
}

std::vector<std::pair<int, int>> iCubPointCloud::getObject2DCoordinates(const Ref<const VectorXd>& bbox, std::size_t stride_u, std::size_t stride_v)
{
    // Create white mask using the current bounding box
    cv::Mat bbox_mask(cam_height_, cam_width_, CV_8UC1, cv::Scalar(0));
    cv::Point tl(int(bbox(0) - bbox(2) / 2.0), int(bbox(1) - bbox(3) / 2.0));
    cv::Point br(int(bbox(0) + bbox(2) / 2.0), int(bbox(1) + bbox(3) / 2.0));
    cv::rectangle(bbox_mask, tl, br, cv::Scalar(255), CV_FILLED);

    // Filter mask taking into account occlusions
    bool is_occlusion_all = false;
    for (auto& occlusion : occlusions_)
    {
        cv::Mat mask;
        bool valid;
        bool is_occlusion;
        std::tie(valid, is_occlusion, mask) = occlusion->removeOcclusion(bbox_mask);

        if (valid)
        {
            bbox_mask = mask.clone();
            is_occlusion_all |= is_occlusion;
        }
    }

    exogenous_data_->setOcclusion(is_occlusion_all);

    // Store a copy of the region of the obtained region of interset
    object_ROI_ = bbox_mask.clone();

    // Find non zero coordinates
    cv::Mat non_zero_coordinates;
    cv::findNonZero(bbox_mask, non_zero_coordinates);

    // Fill coordinates vector
    std::vector<std::pair<int, int>> coordinates;

    for (std::size_t i = 0; i < non_zero_coordinates.total(); i+= (stride_u * stride_v))
    {
        cv::Point& p = non_zero_coordinates.at<cv::Point>(i);
        coordinates.push_back(std::make_pair(p.x, p.y));
    }

    return coordinates;
}


void iCubPointCloud::setDefaultDeprojectionMatrix()
{
    // Compose default deprojection matrix
    default_deprojection_matrix_.resize(3, cam_width_ * cam_height_);
    int i = 0;
    for (std::size_t u = 0; u < cam_width_; u++)
    {
        for (std::size_t v = 0; v < cam_height_; v++)
        {
            default_deprojection_matrix_(0, i) = (u - cam_cx_) / cam_fx_;
            default_deprojection_matrix_(1, i) = (v - cam_cy_) / cam_fy_;
            default_deprojection_matrix_(2, i) = 1.0;

            i++;
        }
    }
}


bool iCubPointCloud::getDepth()
{
    std::string mode = depth_fetch_mode_;
    if (!depth_initialized_)
    {
        // in case a depth was never received
        // it is required to wait at least for the first image in blocking mode
        mode = "new_image";
    }

    ImageOf<PixelFloat>* tmp_depth_in;

    tmp_depth_in = port_depth_in_.read(mode == "new_image");

    if (tmp_depth_in != nullptr)
    {
        depth_image_ = *tmp_depth_in;

        depth_initialized_ = true;
    }

    if (mode == "skip")
        return depth_initialized_ && (tmp_depth_in != nullptr);
    else
        return depth_initialized_;
}


std::tuple<bool, Eigen::MatrixXd, Eigen::VectorXi>
iCubPointCloud::get3DPoints(std::vector<std::pair<int, int>>& coordinates_2d, const float z_threshold)
{
    // Get the camera pose
    yarp::sig::Vector eye_pos_left;
    yarp::sig::Vector eye_att_left;
    yarp::sig::Vector eye_pos_right;
    yarp::sig::Vector eye_att_right;
    Eigen::VectorXd eye_pos(3);
    Eigen::VectorXd eye_att(4);
    if (!gaze_.getCameraPoses(eye_pos_left, eye_att_left, eye_pos_right, eye_att_right))
        return std::make_tuple(false, MatrixXd(), VectorXi());

    if (eye_name_ == "left")
    {
        eye_pos = toEigen(eye_pos_left);
        eye_att = toEigen(eye_att_left);
    }
    else
    {
        eye_pos = toEigen(eye_pos_right);
        eye_att = toEigen(eye_att_right);
    }
    Eigen::AngleAxisd angle_axis(eye_att(3), eye_att.head<3>());

    Eigen::Transform<double, 3, Eigen::Affine> camera_pose;

    // Compose translation
    camera_pose = Translation<double, 3>(eye_pos);

    // Compose rotation
    camera_pose.rotate(angle_axis);

    // Valid points mask
    Eigen::VectorXi valid_points(coordinates_2d.size());

    for (std::size_t i = 0; i < valid_points.size(); i++)
    {
        valid_points(i) = 0;

        float depth_u_v = depth_image_(coordinates_2d[i].first, coordinates_2d[i].second);
        if ((depth_u_v > 0) && (depth_u_v < z_threshold))
            valid_points(i) = 1;
    }

    std::size_t num_valids = valid_points.sum();
    if (num_valids == 0)
        return std::make_tuple(false, MatrixXd(), VectorXi());

    // Compose 3d points with respect to left camera referece frame
    Eigen::MatrixXd points(3, num_valids);
    for (int i = 0, j = 0; i < coordinates_2d.size(); i++)
    {
        if(valid_points(i) == 1)
        {
            const int& u = coordinates_2d[i].first;
            const int& v = coordinates_2d[i].second;

            float depth_u_v = depth_image_(u, v);

            points.col(j) = default_deprojection_matrix_.col(u * cam_height_ + v) * depth_u_v;

            j++;
        }
    }

    // Points with respect to robot root frame
    MatrixXd points_robot = camera_pose * points.colwise().homogeneous();

    return std::make_tuple(true, points_robot, valid_points);
}


void iCubPointCloud::drawObjectROI(cv::Mat& image)
{
    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(object_ROI_, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    // Draw the contours
    cv::drawContours(image, contours, 0, cv::Scalar(0, 255, 0));
}


iCubPointCloudExogenousData::iCubPointCloudExogenousData() :
    bbox_set_(false)
{ }


iCubPointCloudExogenousData:: ~iCubPointCloudExogenousData()
{ }


void iCubPointCloudExogenousData::setBoundingBox(const Ref<const VectorXd>& bounding_box)
{
    bbox_ = bounding_box;

    bbox_set_ = true;
}


std::pair<bool, VectorXd> iCubPointCloudExogenousData::getBoundingBox()
{
    return std::make_pair(bbox_set_, bbox_);
}


void iCubPointCloudExogenousData::setOcclusion(const bool& status)
{
    is_occlusion_ = status;
}


bool iCubPointCloudExogenousData::getOcclusion()
{
    return is_occlusion_;
}


void iCubPointCloudExogenousData::reset()
{
    bbox_set_ = false;

    is_occlusion_ = false;
}

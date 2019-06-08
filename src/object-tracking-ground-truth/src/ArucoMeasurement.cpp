/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ArucoMeasurement.h>
#include <CameraParameters.h>

#include <Eigen/Dense>

#include <yarp/cv/Cv.h>
#include <yarp/eigen/Eigen.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>

using namespace bfl;
using namespace yarp::cv;
using namespace yarp::eigen;
using namespace yarp::os;
using namespace yarp::sig;
using namespace Eigen;


ArucoMeasurement::ArucoMeasurement
(
    std::unique_ptr<Camera> camera,
    const std::string port_prefix,
    const VectorXi& marker_ids,
    const VectorXd& marker_lengths,
    const std::vector<VectorXd>& marker_offsets,
    Eigen::Ref<Eigen::MatrixXd> noise_covariance,
    const bool send_image,
    const bool send_aruco_estimate
) :
    camera_(std::move(camera)),
    marker_lengths_(marker_lengths),
    marker_offsets_(marker_offsets),
    send_image_(send_image),
    send_aruco_estimate_(send_aruco_estimate),
    measurement_(MatrixXd(6, 1)),
    R_(noise_covariance),
    is_measurement_available_(false)
{
    // Open image output port
    if(!port_image_out_.open("/" + port_prefix + "/estimateMarkerImage:o"))
    {
        std::string err = log_ID_ + "CTOR::ERROR\n\tError: cannot open marker estimate image output port.";
        throw(std::runtime_error(err));
    }

    if (send_aruco_estimate_)
    {
        // Open aruco estimate output port
        if(!port_aruco_estimate_out_.open("/" + port_prefix + "/estimateMarker:o"))
        {
            std::string err = log_ID_ + "::CTOR::ERROR\n\tError: cannot open marker estimate output port.";
            throw(std::runtime_error(err));
        }
    }

    // Load camera intrinsic parameters
    bool valid_intrinsic_parameters;
    CameraParameters intrinsic_parameters;
    std::tie(valid_intrinsic_parameters, intrinsic_parameters) = camera_->getIntrinsicParameters();
    if (!valid_intrinsic_parameters)
    {
        std::string err = log_ID_ + "::CTOR::ERROR\n\tError: cannot get camera intrinsic parameters.";
        throw(std::runtime_error(err));
    }

    // Populate cv camera instrincs matrix
    cam_intrinsic_ = cv::Mat(3, 3, CV_64F, 0.0);
    cam_intrinsic_.at<double>(0, 0) = intrinsic_parameters.fx;
    cam_intrinsic_.at<double>(0, 2) = intrinsic_parameters.cx;
    cam_intrinsic_.at<double>(1, 1) = intrinsic_parameters.fy;
    cam_intrinsic_.at<double>(1, 2) = intrinsic_parameters.cy;
    cam_intrinsic_.at<double>(2, 2) = 1.0;

    // Populate cv camera distortion vector
    cam_distortion_ = cv::Mat(1, 4, CV_64F, 0.0);
    cam_distortion_.setTo(cv::Scalar(0.0));

    std::cout << log_ID_ << "CTOR::INFO\n\t Using camera intrinsic:" << std::endl << cam_intrinsic_ << std::endl;
    std::cout << log_ID_ << "CTOR::INFO\n\t Using camera distortion:" << std::endl << cam_distortion_ << std::endl;

    // Configure a standard Aruco dictionary
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);

    // Initialize measurement matrix
    H_ = MatrixXd::Zero(6, 12);
    H_(0, 0) = 1.0;
    H_(1, 1) = 1.0;
    H_(2, 2) = 1.0;
    H_(3, 9) = 1.0;
    H_(4, 10) = 1.0;
    H_(5, 11) = 1.0;

    // Initialize map of marker ids
    // key is the marker_id
    // value is the position in the vector of ids
    for (std::size_t i = 0; i < marker_ids.size(); i++)
        marker_ids_[marker_ids(i)] = i;
}


ArucoMeasurement::~ArucoMeasurement()
{
    // close ports
    port_image_out_.close();

    if (send_aruco_estimate_)
    {
        port_aruco_estimate_out_.close();
    }
}


std::pair<bool, Data> ArucoMeasurement::measure(const Data& data) const
{
    return std::make_pair(is_measurement_available_, measurement_);
}


bool ArucoMeasurement::freeze()
{
    // Get image from camera
    bool valid_image = false;
    cv::Mat image_in;
    std::tie(valid_image, image_in) = camera_->getRgbImage(true);
    if (!valid_image)
        return true;

    // Perform marker detection
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners, rejected;
    cv::aruco::detectMarkers(image_in, dictionary_, corners, ids);

    // Search for a feasible id
    bool found_marker = false;
    int marker_index;
    for (std::size_t i = 0; i < ids.size(); i++)
    {
        std::unordered_map<int, int>::const_iterator match = marker_ids_.find (ids[i]);
        if (match != marker_ids_.end())
        {
            found_marker = true;

            marker_index = match->second;

            break;
        }
    }

    if (!found_marker)
    {
        return true;
    }

    // Estimate pose of markers
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(corners, marker_lengths_(marker_index), cam_intrinsic_, cam_distortion_, rvecs, tvecs);

    if (send_image_)
    {
        // Draw everything was detected for debugging purposes
        for(int i = 0; i < ids.size(); i++)
            cv::aruco::drawAxis(image_in, cam_intrinsic_, cam_distortion_, rvecs[i], tvecs[i], 0.1);

        // Send the image
        ImageOf<PixelRgb>& image_out = port_image_out_.prepare();
        image_out = fromCvMat<PixelRgb>(image_in);
        cv::cvtColor(image_in, image_in, cv::COLOR_RGB2BGR);
        port_image_out_.write();
    }

    // Get camera pose
    bool valid_camera_pose;
    Transform<double, 3, Eigen::Affine> camera_transform;
    std::tie(valid_camera_pose, camera_transform) = camera_->getCameraPose(true);
    if (!valid_camera_pose)
        return true;

    // Compose marker pose
    Vector3d pos_wrt_cam;
    pos_wrt_cam(0) = tvecs.at(0)(0);
    pos_wrt_cam(1) = tvecs.at(0)(1);
    pos_wrt_cam(2) = tvecs.at(0)(2);

    cv::Mat att_wrt_cam_cv;
    cv::Rodrigues(rvecs[0], att_wrt_cam_cv);
    yarp::sig::Matrix att_wrt_cam_yarp(3, 3);

    Matrix3d att_wrt_cam_eigen;
    for (size_t i=0; i<3; i++)
        for (size_t j=0; j<3; j++)
            att_wrt_cam_eigen(i, j) = att_wrt_cam_cv.at<double>(i, j);

    Transform<double, 3, Eigen::Affine> marker_transform;
    marker_transform = Translation<double, 3>(pos_wrt_cam);
    marker_transform.rotate(att_wrt_cam_eigen);

    // Take the correct marker offset
    VectorXd marker_offset = marker_offsets_.at(marker_index);

    // Compose offset pose between marker to body fixed object frame
    Transform<double, 3, Eigen::Affine> offset_transform;
    offset_transform = Translation<double, 3>(Vector3d::Zero());
    offset_transform.rotate(AngleAxisd(AngleAxisd(marker_offset(3), Vector3d::UnitX()) *
				       AngleAxisd(marker_offset(4), Vector3d::UnitY()) *
				       AngleAxisd(marker_offset(5), Vector3d::UnitZ())));

    // Compose transform from root frame to object frame
    Transform<double, 3, Eigen::Affine> transform  = camera_transform * marker_transform * offset_transform;

    // Compose the actual measurement
    measurement_.col(0).head<3>() = transform * marker_offset.head<3>().homogeneous();
    measurement_.col(0).tail<3>() = transform.rotation().eulerAngles(2, 1, 0);

    if (send_aruco_estimate_)
    {
        VectorXd pose(7);
        pose.head<3>() = measurement_.col(0).head<3>();

        AngleAxisd orientation(transform.rotation());

        pose.segment(3, 3) = orientation.axis();
        pose(6) = orientation.angle();

        yarp::sig::Vector& pose_yarp = port_aruco_estimate_out_.prepare();
        pose_yarp.resize(7);
        toEigen(pose_yarp) = pose;
        port_aruco_estimate_out_.write();
    }

    is_measurement_available_ = true;

    return true;
}

std::pair<bool, bfl::Data> ArucoMeasurement::innovation(const bfl::Data& predicted_measurements, const bfl::Data& measurements) const
{
    MatrixXd innovation = -(any::any_cast<MatrixXd>(predicted_measurements).colwise() - any::any_cast<MatrixXd>(measurements).col(0));

    // Wrap Euler angles
    innovation.bottomRows(3) = (std::complex<double>(0.0,1.0) * innovation.bottomRows(3)).array().exp().arg();

    return std::make_pair(true, std::move(innovation));
}


Eigen::MatrixXd ArucoMeasurement::getMeasurementMatrix() const
{
    return H_;
}


std::pair<bool, MatrixXd> ArucoMeasurement::getNoiseCovarianceMatrix() const
{
    return std::make_pair(true, R_);
}


std::pair<std::size_t, std::size_t> ArucoMeasurement::getOutputSize() const
{
    /* Three cartesian coordinates, three euler angles (ZXY parametrization). */
    return std::make_pair(3, 3);
}


bool ArucoMeasurement::setProperty(const std::string& property)
{
    if (property == "reset")
        is_measurement_available_ = false;

    return true;
}

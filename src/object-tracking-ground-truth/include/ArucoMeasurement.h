#ifndef ARUCOMEASUREMENT_H
#define ARUCOMEASUREMENT_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <BayesFilters/Data.h>
#include <BayesFilters/LinearMeasurementModel.h>

#include <Eigen/Dense>

#include <GazeController.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>


class ArucoMeasurement : public bfl::LinearMeasurementModel
{
public:
    ArucoMeasurement(const std::string port_prefix, const std::string eye_name, const Eigen::Ref<Eigen::VectorXd> marker_offset, const double marker_length, Eigen::Ref<Eigen::MatrixXd> noise_covariance, const bool send_image, const bool send_aruco_estimate);

    virtual ~ArucoMeasurement();

    std::pair<bool, bfl::Data> measure() const override;

    bool freezeMeasurements() override;

    std::pair<bool, bfl::Data> innovation(const bfl::Data& predicted_measurements, const bfl::Data& measurements) const override;

    Eigen::MatrixXd getMeasurementMatrix() const;

    std::pair<bool, Eigen::MatrixXd> getNoiseCovarianceMatrix() const;

    std::pair<std::size_t, std::size_t> getOutputSize() const override;

    bool setProperty(const std::string& property) override;

protected:
    std::pair<bool, Eigen::Transform<double, 3, Eigen::Affine>> getCameraPose();

    const std::string eye_name_;

    cv::Ptr<cv::aruco::Dictionary> dictionary_;

    cv::Mat cam_intrinsic_;

    cv::Mat cam_distortion_;

    cv::Mat image_with_marker_;

    Eigen::VectorXd marker_offset_;

    double marker_length_;

    /**
     * Gaze controller.
     */
    GazeController gaze_;

    /**
     * Latest Object pose. This pose is used as a measurement.
     */
    Eigen::MatrixXd measurement_;
    bool is_measurement_available_;

    /*
     * Linear measurement matrix
     */
    Eigen::MatrixXd H_;

    /*
     * Noise covariance matrix
     */
    Eigen::MatrixXd R_;

    /**
     * Image input / output.
     */
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> port_image_in_;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> port_image_out_;

    yarp::os::BufferedPort<yarp::sig::Vector> port_aruco_estimate_out_;

    bool send_image_;

    bool send_aruco_estimate_;

    const std::string log_ID_ = "[ARUCOMEASUREMENT]";
};

#endif /* ARUCOMEASUREMENT_H */

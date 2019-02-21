#ifndef ARUCOMEASUREMENT_H
#define ARUCOMEASUREMENT_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <BayesFilters/LinearMeasurementModel.h>

#include <GazeController.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>


class ArucoMeasurement : bfl::LinearMeasurementModel
{
public:
    ArucoMeasurement(const std::string port_prefix);

    virtual ~ArucoMeasurement();

    std::pair<bool, bfl::Data> measure() const override;

    bool freezeMeasurements() override;

    std::pair<std::size_t, std::size_t> getOutputSize() const override;

protected:
    const std::string eye_name_;

    cv::Ptr<cv::aruco::Dictionary> dictionary_;

    cv::Mat cam_intrinsic_;

    cv::Mat cam_distortion_;

    cv::Mat image_with_marker_;

    /**
     * Gaze controller.
     */
    GazeController gaze_;

    /**
     * Latest Object pose. This pose is used as a measurement.
     */
    Eigen::VectorXd measurement_;

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

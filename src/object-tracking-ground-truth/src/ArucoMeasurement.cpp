#include <ArucoMeasurement.h>

#include <yarp/os/ResourceFinder.h>

using namespace yarp::os;


bool ArucoMeasurement::ArucoMeasurement
(
    const std::string port_prefix,
    const std::string eye_name,
    const bool send_image,
    const bool send_aruco_estimate
) :
    eye_name_(eye_name),
    send_image_(send_image),
    send_aruco_estimate_(send_aruco_estimate),
    gaze_(port_prefix)
{
    // Open camera  input port
    if(!port_image_in_.open("/" + port_prefix + "/cam/" + eye_name + ":i"))
    {
        std::string err = log_ID_ + "::CTOR::ERROR\n\tError: cannot open " + eye_name + " camera input port.";
        throw(std::runtime_error(err));
    }

    // Open image output port
    if(!port_image_out_.open("/" + port_prefix + "/marker_estimate/" + eye_name + ":o"))
    {
        std::string err = log_ID_ + "CTOR::ERROR\n\tError: cannot open marker estimate image output port.";
        throw(std::runtime_error(err));
    }

    if (send_aruco_estimate_)
    {
        // Open aruco estimate output port
        if(!port_aruco_estimate_out_.open("/" + port_prefix + "/marker_estimate/" + eye_name + ":o"))
        {
            std::string err = log_ID_ + "::CTOR::ERROR\n\tError: cannot open marker estimate output port.";
            throw(std::runtime_error(err));
        }
    }

    // Find the path of the camera intrinsic parameters
    ResourceFinder rf_ground_truth;
    rf_ground_truth.setVerbose(true);
    rf_ground_truth.setDefaultContext("object-tracking-ground-truth");
    std::string intrinsic_path = rf_ground_truth.findFile("opencv_icub_intrinsics_" + eye_name);

    // Load camera intrinsic parameters
    cv::FileStorage fs(intrinsic_path, cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        std::string err = log_ID_ + "::CTOR::ERROR\n\tError: cannot open camera intrinsic parameters file.";
        throw(std::runtime_error(err));
    }

    fs["camera_matrix"] >> cam_intrinsic_;
    fs["distortion_coefficients"] >> cam_distortion_;

    // If available use camera intrinsics from the gaze controller
    double fx;
    double fy;
    double cx;
    double cy;
    bool valid_intrinsics = gaze_.getCameraIntrinsics(eye_name_, fx, fy, cx, cy);
    if (valid_intrinsics)
    {
        cam_intrinsic.at<double>(0, 0) = fx;
        cam_intrinsic.at<double>(0, 2) = cx;
        cam_intrinsic.at<double>(1, 1) = fy;
        cam_intrinsic.at<double>(1, 2) = cy;
    }

    // Configure a standard Aruco dictionary
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);

    // Missing code required to
    // - read offset from object surface to object center in body fixed frame from configuration file
}


ArucoMeasurement::~ArucoMeasurement()
{
    // close ports
    port_image_in_.close();

    port_image_out_.close();

    if (send_aruco_estimate_)
    {
        port_aruco_estimate_out.close();
    }
}


bool ArucoMeasurement::freezeMeasurements()
{
    ImageOf<PixelRgb>* image_in;
    image_in = port_image_in_.read(true);

    if (image_in == nullptr)
        return false;

    // Prepare output image
    ImageOf<PixelRgb>& image_out = port_image_out_.prepare();

    // Copy input to output and wrap around a cv::Mat
    image_out = *image_in;
    cv::Mat image = yarp::cv::toCvMat(image_out);

    // Perform marker detection
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners, rejected;
    cv::aruco::detectMarkers(image, dictionary_, corners, ids);

    // Estimate pose of markers
    std::vector<cv::Mat> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(corners, 0.05, cam_intrinsic_, cam_distortion_, rvecs, tvecs);

    if (send_image_)
    {
        // Draw axis for each marker found
        for(int i = 0; i < ids.size(); i++)
            cv::aruco::drawAxis(image, cam_intrinsic_, cam_distortion_, rvecs[i], tvecs[i], 0.1);

        // Send the image
        port_image_out_.write();
    }

        // transformation matrix
    // from robot root to camera
    yarp::sig::Matrix root_to_cam(4, 4);
    root_to_cam.zero();
    root_to_cam(3, 3) = 1.0;

    root_to_cam(0, 3) = camera_pos[0];
    root_to_cam(1, 3) = camera_pos[1];
    root_to_cam(2, 3) = camera_pos[2];

    root_to_cam.setSubmatrix(yarp::math::axis2dcm(camera_att).submatrix(0, 2, 0, 2),
                             0, 0);

    // TODO: MISSING CODE required to
    // - rotate body fixed reference frame taking into account convention of aruco
    // - apply fixed offset from object surface to center of object
    // - rewrite with origin in root frame and expressed in root frame
    // - save measurement (x,y,z,y,p,r) and send to port (x,y,z,axis,angle) if required
    //
    // // transformation matrix
    // // from camera to corner of the object
    // yarp::sig::Matrix cam_to_obj(4,4);
    // cam_to_obj.zero();
    // cam_to_obj(3, 3) = 1.0;

    // cam_to_obj(0, 3) = pos_wrt_cam.at<double>(0, 0);
    // cam_to_obj(1, 3) = pos_wrt_cam.at<double>(1, 0);
    // cam_to_obj(2, 3) = pos_wrt_cam.at<double>(2, 0);

    // cv::Mat att_wrt_cam_matrix;
    // cv::Rodrigues(att_wrt_cam, att_wrt_cam_matrix);
    // yarp::sig::Matrix att_wrt_cam_yarp(3, 3);
    // for (size_t i=0; i<3; i++)
    //     for (size_t j=0; j<3; j++)
    //         att_wrt_cam_yarp(i, j) = att_wrt_cam_matrix.at<double>(i, j);
    // cam_to_obj.setSubmatrix(att_wrt_cam_yarp, 0, 0);

    // // compose transformations
    // yarp::sig::Matrix homog = root_to_cam * cam_to_obj;

    // // convert to a vector
    // est_pose = homogToVector(homog);

    // if (is_estimate_available)
    // {
    //     // override z using the initial pose
    //     est_pose[2] = initial_pose[2];

    //     // override pitch and roll using the initial pose
    //     est_pose.setSubvector(4, initial_pose.subVector(4, 5));
    // }

    // // pick the center of the object
    // yarp::sig::Vector corner_to_center(4);
    // corner_to_center[0] = obj_width / 2.0;
    // corner_to_center[1] = obj_depth / 2.0;
    // corner_to_center[2] = -obj_height / 2.0;
    // corner_to_center[3] = 1.0;

    // yarp::sig::Vector center = est_pose_homog * corner_to_center;
    // est_pose_homog.setCol(3, center);

    return false;
}


std::pair<std::size_t, std::size_t> getOutputSize()
{
    /* Three cartesian coordinates, three euler angles (ZXY parametrization). */
    return std::make_pair(3, 3);
}

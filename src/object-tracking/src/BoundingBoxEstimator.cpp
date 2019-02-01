#include <BoundingBoxEstimator.h>

#include <yarp/cv/Cv.h>
#include <yarp/sig/Vector.h>

#include <iostream>

using namespace Eigen;
using namespace yarp::os;
using namespace yarp::sig;

BoundingBoxEstimator::BoundingBoxEstimator
(
    const BoundingBoxEstimator::BBox initial_bbox,
    const std::string port_prefix,
    const std::string eye_name,
    const std::string obj_mesh_file,
    const std::string sicad_shader_path,
    const std::string IOL_object_name,
    const bool send_mask,
    const Eigen::Ref<const Eigen::MatrixXd>& initial_covariance,
    const Eigen::Ref<const Eigen::MatrixXd>& process_noise_covariance,
    const Eigen::Ref<const Eigen::MatrixXd>& measurement_noise_covariance
) :
    BoundingBoxEstimator
    (
        port_prefix,
        eye_name,
        obj_mesh_file,
        sicad_shader_path,
        IOL_object_name,
        send_mask,
        initial_covariance,
        process_noise_covariance,
        measurement_noise_covariance
    )
{
    // Initialize center, width and height
    auto top_left = initial_bbox.first;
    auto bottom_right = initial_bbox.second;
    mean_0_.resize(4);
    mean_0_(0) = (top_left.first + bottom_right.first) / 2.0;
    mean_0_(1) = (top_left.second + bottom_right.second) / 2.0;
    mean_0_(2) = (bottom_right.first - top_left.first);
    mean_0_(3) = (bottom_right.second - top_left.second);

    // Set flags
    user_provided_mean_0_ = true;
    is_initialized_ = true;

    // Initialize bounding box
    corr_bbox_.mean() = mean_0_;
}


BoundingBoxEstimator::BoundingBoxEstimator
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
) :
    pred_bbox_(4),
    corr_bbox_(4),
    eye_name_(eye_name),
    IOL_object_name_(IOL_object_name),
    send_mask_(send_mask),
    user_provided_mean_0_(false),
    is_initialized_(false),
    is_exogenous_initialized_(false),
    is_proj_bbox_initialized_(false),
    is_object_pose_initialized_(false),
    gaze_(port_prefix),
    cov_0_(initial_covariance),
    Q_(process_noise_covariance),
    R_(measurement_noise_covariance)
{
    // Open RPC port to OPC required to get "measurements" of the bounding box
    if (!(opc_rpc_client_.open("/" + port_prefix + "/opc/rpc:o")))
    {
        std::string err = "ICUBPOINTCLOUD::CTOR::ERROR\n\tError: cannot open OPC rpc port.";
        throw(std::runtime_error(err));
    }

    if (send_mask_)
    {
        // Open camera input port.
        if(!port_image_in_.open("/" + port_prefix + "/cam/" + eye_name + ":i"))
        {
            std::string err = "VIEWER::CTOR::ERROR\n\tError: cannot open " + eye_name + " camera input port.";
            throw(std::runtime_error(err));
        }

        // Open image output port.
        if(!port_mask_image_out_.open("/" + port_prefix + "/mask:o"))
        {
            std::string err = "VIEWER::CTOR::ERROR\n\tError: cannot open mask output port.";
            throw(std::runtime_error(err));
        }
    }

    // Get iCub cameras intrinsics parameters
    if (!gaze_.getCameraIntrinsics(eye_name, cam_fx_, cam_fy_, cam_cx_, cam_cy_))
    {
        std::string err = "ICUBPOINTCLOUD::CTOR::ERROR\n\tError: cannot retrieve iCub camera intrinsicse.";
        throw(std::runtime_error(err));
    }

    // Configure superimposition engine
    SICAD::ModelPathContainer mesh_path;
    mesh_path.emplace("object", obj_mesh_file);
    object_sicad_ = std::unique_ptr<SICAD>
        (
            new SICAD(mesh_path,
                      cam_width_,
                      cam_height_,
                      cam_fx_,
                      cam_fy_,
                      cam_cx_,
                      cam_cy_,
                      1,
                      sicad_shader_path,
                      {1.0, 0.0, 0.0, static_cast<float>(M_PI)})
        );

    // Initialize state covariance
    corr_bbox_.covariance() = cov_0_;

    steady_state_counter_ = 0;
    steady_state_threshold_ = 200;
}


BoundingBoxEstimator::~BoundingBoxEstimator()
{
    // Close ports
    opc_rpc_client_.close();

    if (send_mask_)
    {
        port_image_in_.close();
        port_mask_image_out_.close();
    }
}


void BoundingBoxEstimator::step()
{
    while(!is_initialized_)
    {
        bool valid_measure;
        VectorXd mean_0;
        std::tie(valid_measure, mean_0) = measure();

        if (valid_measure)
        {
            is_initialized_ = true;
            corr_bbox_.mean() = mean_0;
        }
    }

    predict();
    correct();
}


Eigen::VectorXd BoundingBoxEstimator::getEstimate()
{
    return corr_bbox_.mean();
}


void BoundingBoxEstimator::reset()
{
    is_initialized_ = false;
    is_exogenous_initialized_ = false;
    is_proj_bbox_initialized_ = false;
    is_object_pose_initialized_ = false;

    // If user provided a initial mean
    if (user_provided_mean_0_)
    {
        corr_bbox_.mean() = mean_0_;

        is_initialized_ = true;
    }

    corr_bbox_.covariance() = cov_0_;

    steady_state_counter_ = 0;
}


void BoundingBoxEstimator::setObjectPose(const Eigen::Ref<const Eigen::VectorXd>& pose)
{
    object_3d_pose_ = pose;

    is_object_pose_initialized_ = true;
}


void BoundingBoxEstimator::predict()
{
    VectorXd input = evalExogenousInput();

    // state transition
    pred_bbox_.mean() = corr_bbox_.mean() + input;

    // covariance transition
    pred_bbox_.covariance() += Q_;
}


void BoundingBoxEstimator::correct()
{
    bool valid_measure;
    VectorXd measured_bbox;
    std::tie(valid_measure, measured_bbox) = measure();

    if (!valid_measure)
    {
        corr_bbox_ = pred_bbox_;

        return;
    }

    MatrixXd Py = pred_bbox_.covariance() + R_;

    MatrixXd K = pred_bbox_.covariance() * Py.inverse();

    corr_bbox_.mean() = pred_bbox_.mean() + K * (measured_bbox - pred_bbox_.mean());

    corr_bbox_.covariance() = pred_bbox_.covariance() - K * Py * K.transpose();
}


VectorXd BoundingBoxEstimator::evalExogenousInput()
{
    if (!updateObjectMask())
        return VectorXd::Zero(4);

    // Find projected bounding box
    cv::Mat points;
    cv::findNonZero(object_mask_, points);
    cv::Rect bbox_rect = cv::boundingRect(points);

    // Form vector containing center, width and height of projected bounding box
    VectorXd curr_bbox(4);
    curr_bbox(0) = bbox_rect.x + bbox_rect.width / 2.0;
    curr_bbox(1) = bbox_rect.y + bbox_rect.height / 2.0;
    curr_bbox(2) = bbox_rect.width;
    curr_bbox(3) = bbox_rect.height;

    VectorXd exog_input = VectorXd::Zero(4);

    if (is_proj_bbox_initialized_)
    {
        // Evaluate finite differences
        VectorXd delta = curr_bbox - proj_bbox_;

        if (!is_exogenous_initialized_)
        {
            if (steady_state_counter_ > steady_state_threshold_)
            {
                is_exogenous_initialized_ = true;

                // Evaluate width and height ratio
                bbox_width_ratio_ = corr_bbox_.mean(2) / bbox_rect.width;
                bbox_height_ratio_ = corr_bbox_.mean(3) / bbox_rect.height;
            }

        }
        else
        {
            exog_input = delta;
            exog_input(2) *= bbox_width_ratio_;
            exog_input(3) *= bbox_height_ratio_;
        }
    }

    // Store projected bounding box for next iteration
    proj_bbox_ = curr_bbox;
    is_proj_bbox_initialized_ = true;

    // Increment counter
    steady_state_counter_++;

    return exog_input;
}


std::pair<bool, VectorXd> BoundingBoxEstimator::measure()
{
    /*
     * Get object bounding box from OPC module given object name.
     *
     * Adapted from https://github.com/robotology/point-cloud-read
     */
    VectorXd bbox(4);

    // Command message format is: [ask] (("prop0" "<" <val0>) || ("prop1" ">=" <val1>) ...)
    Bottle cmd, reply;
    cmd.addVocab(Vocab::encode("ask"));
    Bottle &content = cmd.addList().addList();
    content.addString("name");
    content.addString("==");
    content.addString(IOL_object_name_);

    if(!opc_rpc_client_.write(cmd,reply))
        return std::make_pair(false, VectorXd());

    // reply message format: [nack]; [ack] ("id" (<num0> <num1> ...))
    if (reply.size()>1)
    {
        //  verify that first element is "ack"
        if (reply.get(0).asVocab() == Vocab::encode("ack"))
        {
            //  get list of all id's of objects named obj_name
            if (Bottle* id_field = reply.get(1).asList())
            {
                if (Bottle* id_values = id_field->get(1).asList())
                {
                    //  if there are more objects under the same name, pick the first one
                    int id = id_values->get(0).asInt();

                    //  get the actual bounding box
                    //  command message format:  [get] (("id" <num>) (propSet ("prop0" "prop1" ...)))
                    cmd.clear();
                    cmd.addVocab(Vocab::encode("get"));
                    Bottle& content = cmd.addList();
                    Bottle& list_bid = content.addList();
                    list_bid.addString("id");
                    list_bid.addInt(id);
                    Bottle& list_propSet = content.addList();
                    list_propSet.addString("propSet");
                    Bottle& list_items = list_propSet.addList();
                    list_items.addString("position_2d_left");
                    Bottle reply_prop;
                    opc_rpc_client_.write(cmd,reply_prop);

                    //reply message format: [nack]; [ack] (("prop0" <val0>) ("prop1" <val1>) ...)
                    if (reply_prop.get(0).asVocab() == Vocab::encode("ack"))
                    {
                        if (Bottle* prop_field = reply_prop.get(1).asList())
                        {
                            if (Bottle* position_2d_bb = prop_field->find("position_2d_left").asList())
                            {
                                std::pair<int, int> top_left;
                                std::pair<int, int> bottom_right;

                                //  position_2d_left contains x,y of top left and x,y of bottom right
                                top_left.first      = position_2d_bb->get(0).asInt();
                                top_left.second     = position_2d_bb->get(1).asInt();
                                bottom_right.first  = position_2d_bb->get(2).asInt();
                                bottom_right.second = position_2d_bb->get(3).asInt();

                                // initialize center, width and height
                                bbox(0) = (top_left.first + bottom_right.first) / 2.0;
                                bbox(1) = (top_left.second + bottom_right.second) / 2.0;
                                bbox(2) = (bottom_right.first - top_left.first);
                                bbox(3) = (bottom_right.second - top_left.second);

                                return std::make_pair(true, bbox);
                            }
                        }
                    }
                }
            }
        }
    }

    return std::make_pair(false, VectorXd());
}


void BoundingBoxEstimator::sendObjectMask(ImageOf<PixelRgb>& camera_image)
{
    if (!(object_mask_.empty()))
    {
        // Prepare output image
        ImageOf<PixelRgb>& image_out = port_mask_image_out_.prepare();

        // Copy input to output and wrap around a cv::Mat
        image_out = camera_image;
        cv::Mat image = yarp::cv::toCvMat(image_out);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        // Draw mask on top of image
        cv::Mat mask = object_mask_.clone();

        cv::cvtColor(mask, mask, cv::COLOR_GRAY2RGB);
        cv::bitwise_or(image, mask, image);

        port_mask_image_out_.write();
    }
}


bool BoundingBoxEstimator::updateObjectMask()
{
    if (!is_object_pose_initialized_)
        return false;

    // Convert state to Superimpose::ModelPose format
    Superimpose::ModelPose si_object_pose;
    si_object_pose.resize(7);

    // Cartesian coordinates
    si_object_pose[0] = object_3d_pose_(0);
    si_object_pose[1] = object_3d_pose_(1);
    si_object_pose[2] = object_3d_pose_(2);

    // Convert from Euler ZYX to axis/angle
    AngleAxisd angle_axis(AngleAxisd(object_3d_pose_(9), Vector3d::UnitZ()) *
                          AngleAxisd(object_3d_pose_(10), Vector3d::UnitY()) *
                          AngleAxisd(object_3d_pose_(11), Vector3d::UnitX()));
    si_object_pose[3] = angle_axis.axis()(0);
    si_object_pose[4] = angle_axis.axis()(1);
    si_object_pose[5] = angle_axis.axis()(2);
    si_object_pose[6] = angle_axis.angle();

    // Set pose
    Superimpose::ModelPoseContainer object_pose_container;
    object_pose_container.emplace("object", si_object_pose);

    yarp::sig::Vector eye_pos_left;
    yarp::sig::Vector eye_att_left;
    yarp::sig::Vector eye_pos_right;
    yarp::sig::Vector eye_att_right;
    if (!gaze_.getCameraPoses(eye_pos_left, eye_att_left, eye_pos_right, eye_att_right))
    {
        // Not updating the bounding box since the camera pose is not available
        return false;
    }

    // Project mesh onto the camera plane
    // The shader is designed to have the mesh rendered as a white surface project onto the camera plane
    bool valid_superimpose = false;
    if (eye_name_ == "left")
        valid_superimpose = object_sicad_->superimpose(object_pose_container, eye_pos_left.data(), eye_att_left.data(), object_mask_);
    else if (eye_name_ == "right")
        valid_superimpose = object_sicad_->superimpose(object_pose_container, eye_pos_right.data(), eye_att_right.data(), object_mask_);

    if (!valid_superimpose)
    {
        std::string err = "BOUNDINGBOXESTIMATOR::UPDATEOBJECTMASK::ERROR\n\tError: cannot superimpose mesh onto the camera plane.";
        throw(std::runtime_error(err));
    }

    // Convert to gray scale
    cv::cvtColor(object_mask_, object_mask_, CV_BGR2GRAY);

    return true;
}

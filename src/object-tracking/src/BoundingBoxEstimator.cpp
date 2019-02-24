#include <BoundingBoxEstimator.h>

#include <yarp/cv/Cv.h>
#include <yarp/eigen/Eigen.h>
#include <yarp/sig/Vector.h>

#include <iostream>

using namespace bfl;
using namespace Eigen;
using namespace yarp::eigen;
using namespace yarp::os;
using namespace yarp::sig;

BoundingBoxEstimator::BoundingBoxEstimator
(
    const BoundingBoxEstimator::BBox initial_bbox,
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
) :
    BoundingBoxEstimator
    (
        number_components,
        port_prefix,
        eye_name,
        obj_mesh_file,
        sicad_shader_path,
        IOL_object_name,
        IOL_bbox_scale,
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
    for (std::size_t i = 0; i < corr_bbox_.components; i++)
        corr_bbox_.mean(i) = mean_0_;
}


BoundingBoxEstimator::BoundingBoxEstimator
(
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
) :
    pred_bbox_(number_components, 4),
    corr_bbox_(number_components, 4),
    eye_name_(eye_name),
    IOL_object_name_(IOL_object_name),
    IOL_bbox_scale_(IOL_bbox_scale),
    send_mask_(send_mask),
    user_provided_mean_0_(false),
    extractor_(4),
    is_initialized_(false),
    is_exogenous_initialized_(false),
    is_object_pose_initialized_(false),
    is_hand_exogenous_initialized_(false),
    gaze_(port_prefix),
    cov_0_(initial_covariance),
    Q_(process_noise_covariance),
    R_(measurement_noise_covariance)
{
    // Open RPC port to OPC required to get "measurements" of the bounding box
    if (!(opc_rpc_client_.open("/" + port_prefix + "/opc/rpc:o")))
    {
        std::string err = log_ID_ + "::CTOR::ERROR\n\tError: cannot open OPC rpc port.";
        throw(std::runtime_error(err));
    }

    // Try to open the hand pose input port
    if (!(hand_pose_port_in_.open("/" + port_prefix + "/hand_pose:i")))
    {
        std::string err = log_ID_ + "::CTOR::ERROR\n\tError: cannot open hand pose input port.";
        throw(std::runtime_error(err));
    }

    if (send_mask_)
    {
        // Open camera input port.
        if(!port_image_in_.open("/" + port_prefix + "/cam/" + eye_name + ":i"))
        {
            std::string err = log_ID_ + "::CTOR::ERROR\n\tError: cannot open " + eye_name + " camera input port.";
            throw(std::runtime_error(err));
        }

        // Open image output port.
        if(!port_mask_image_out_.open("/" + port_prefix + "/mask:o"))
        {
            std::string err = log_ID_ + "CTOR::ERROR\n\tError: cannot open mask output port.";
            throw(std::runtime_error(err));
        }
    }

    // Get iCub cameras intrinsics parameters
    if (!gaze_.getCameraIntrinsics(eye_name, cam_fx_, cam_fy_, cam_cx_, cam_cy_))
    {
        std::string err = log_ID_ + "CTOR::ERROR\n\tError: cannot retrieve iCub camera intrinsicse.";
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
                      number_components,
                      sicad_shader_path,
                      {1.0, 0.0, 0.0, static_cast<float>(M_PI)})
        );

    // The sicad engine may be able to render less images than the requested number of particles
    number_components_ = object_sicad_->getTilesNumber();
    pred_bbox_.components = number_components_;
    corr_bbox_.components = number_components_;

    // Initialize state covariance
    for (std::size_t i = 0; i < corr_bbox_.components; i++)
        corr_bbox_.covariance(i) = cov_0_;

    steady_state_counter_ = 0;
    steady_state_threshold_ = 75;

    // Initialize estimate extractor
    extractor_.setMethod(EstimatesExtraction::ExtractionMethod::emode);
    extractor_.setMobileAverageWindowSize(10);

    // Reset flag
    enable_object_feedforward_ = true;
    enable_hand_feedforward_ = false;
    enable_hand_feedforward_first_time_ = false;
}


BoundingBoxEstimator::~BoundingBoxEstimator()
{
    // Close ports
    opc_rpc_client_.close();
    hand_pose_port_in_.close();

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
            for (std::size_t i = 0; i < corr_bbox_.components; i++)
                corr_bbox_.mean(i) = mean_0;

            is_initialized_ = true;
        }
    }

    predict();
    correct();
}


Eigen::VectorXd BoundingBoxEstimator::getEstimate()
{
    return corr_bbox_.mean(0);
}


Eigen::VectorXd BoundingBoxEstimator::getEstimate(const Ref<const VectorXd>& weights)
{
    // VectorXd estimate;
    // std::tie(std::ignore, estimate) = extractor_.extract(corr_bbox_.mean().leftCols(corr_bbox_.components), weights);

    // return estimate;
    return corr_bbox_.mean(0);
}


void BoundingBoxEstimator::reset()
{
    is_initialized_ = false;
    is_exogenous_initialized_ = false;
    is_object_pose_initialized_ = false;
    is_hand_exogenous_initialized_ = false;
    enable_hand_feedforward_first_time_ = false;
    enable_object_feedforward_= true;
    enable_hand_feedforward_ = false;

    // If user provided a initial mean
    if (user_provided_mean_0_)
    {
        for (std::size_t i = 0; i < corr_bbox_.components; i++)
            corr_bbox_.mean(i) = mean_0_;

        is_initialized_ = true;
    }

    for (std::size_t i = 0; i < corr_bbox_.components; i++)
        corr_bbox_.covariance() = cov_0_;

    extractor_.clear();

    steady_state_counter_ = 0;
}


void BoundingBoxEstimator::setObjectPose(const Ref<const MatrixXd>& pose)
{
    VectorXd weights(1);
    weights(0) = 1.0;

    setObjectPose(pose, weights);
}

void BoundingBoxEstimator::setObjectPose(const Ref<const MatrixXd>& pose, const Ref<const VectorXd>& weights)
{
    object_3d_pose_ = pose;

    object_3d_pose_weights_ = weights;

    is_object_pose_initialized_ = true;
}


std::size_t BoundingBoxEstimator::getNumberComponents()
{
    return number_components_;
}


void BoundingBoxEstimator::enableObjectFeedforward(const bool& enable)
{
    enable_object_feedforward_ = enable;
}


void BoundingBoxEstimator::enableHandFeedforward(const bool& enable)
{
    enable_hand_feedforward_ = enable;
}


void BoundingBoxEstimator::resampleParticles(const VectorXi& parents)
{
    // GaussianMixture resampled_pred(pred_bbox_.components, 4);
    // GaussianMixture resampled_corr(pred_bbox_.components, 4);
    // MatrixXd resampled_hand_object_rotation(relative_hand_object_rotation_.rows(), relative_hand_object_rotation_.cols());

    // for (std::size_t i = 0; i < pred_bbox_.components; i++)
    // {
    //     resampled_pred.mean(i) = pred_bbox_.mean(parents(i));
    //     resampled_pred.covariance(i) = pred_bbox_.covariance(parents(i));

    //     resampled_corr.mean(i) = corr_bbox_.mean(parents(i));
    //     resampled_corr.covariance(i) = corr_bbox_.covariance(parents(i));

    //     resampled_hand_object_rotation.col(i) = relative_hand_object_rotation_.col(parents(i));
    // }

    // pred_bbox_ = resampled_pred;
    // corr_bbox_ = resampled_corr;
    // relative_hand_object_rotation_ = resampled_hand_object_rotation;
}


void BoundingBoxEstimator::predict()
{
    // state transition
    pred_bbox_.mean() = corr_bbox_.mean();

    if (enable_hand_feedforward_first_time_ || enable_hand_feedforward_)
    {
        pred_bbox_.mean().leftCols(pred_bbox_.components) += evalHandExogenousInput();

	// this is required since even if there is a firm grasp
	// sometimes contacts are missing!
	enable_hand_feedforward_first_time_ = true;
    }
    // else
    // {
    //     // Assume that hand feedforward has to be reset
    //     is_hand_exogenous_initialized_ = false;
    // }

    // covariance transition
    for (std::size_t i = 0; i < pred_bbox_.components; i++)
        pred_bbox_.covariance(i) += Q_;
}


void BoundingBoxEstimator::correct()
{
    // bool valid_measure;
    // VectorXd measured_bbox;
    // std::tie(valid_measure, measured_bbox) = measure();

    // if (!valid_measure)
    // {
    corr_bbox_ = pred_bbox_;

    //     return;
    // }

    // for (std::size_t i = 0; i < pred_bbox_.components; i++)
    // {
    //     MatrixXd Py = pred_bbox_.covariance(i) + R_;

    //     MatrixXd K = pred_bbox_.covariance(i) * Py.inverse();

    //     corr_bbox_.mean(i) = pred_bbox_.mean(i) + K * (measured_bbox - pred_bbox_.mean(i));

    //     corr_bbox_.covariance(i) = pred_bbox_.covariance(i) - K * Py * K.transpose();
    // }
}


Eigen::MatrixXd BoundingBoxEstimator::evalHandExogenousInput()
{
    // Get the hand pose
    yarp::sig::Vector* hand_pose_yarp = hand_pose_port_in_.read(false);
    if (hand_pose_yarp == nullptr)
        return MatrixXd::Zero(4, pred_bbox_.components);
    VectorXd curr_hand_pose = toEigen(*hand_pose_yarp);

    // Get the current stamp
    Stamp curr_stamp;
    hand_pose_port_in_.getEnvelope(curr_stamp);

    MatrixXd delta = MatrixXd::Zero(4, pred_bbox_.components);

    if (is_hand_exogenous_initialized_)
    {
        // if ((curr_stamp.getTime() - hand_pose_stamp_.getTime()) < 1)
        // {
            // Evaluate relative motion of the hand
            // Matrix3d hand_rot_prev = AngleAxisd(hand_pose_(6), hand_pose_.segment(3, 3)).toRotationMatrix();
            Matrix3d hand_rot_curr = AngleAxisd(curr_hand_pose(6), curr_hand_pose.segment(3, 3)).toRotationMatrix();
            // Matrix3d relative_rot = hand_rot_prev.transpose() * hand_rot_curr;
            Vector3d relative_pos = curr_hand_pose.segment(0, 3) - hand_pose_.segment(0, 3);

            // Perturb previously stored object poses
            object_3d_pose_perturbed_.topRows<3>().colwise() += relative_pos;

            for (std::size_t i = 0; i < pred_bbox_.components; i++)
            {
                // Matrix3d object_rot_prev = (AngleAxisd(object_3d_pose_perturbed_.col(i)(9), Vector3d::UnitZ()) *
                //                             AngleAxisd(object_3d_pose_perturbed_.col(i)(10), Vector3d::UnitY()) *
                //                             AngleAxisd(object_3d_pose_perturbed_.col(i)(11), Vector3d::UnitX())).toRotationMatrix();
                // Matrix3d perturbed_rot = object_rot_prev * relative_rot;

                // Matrix3d relative_rotation = (AngleAxisd(relative_hand_object_rotation_.col(i)(9), Vector3d::UnitZ()) *
                //                               AngleAxisd(relative_hand_object_rotation_.col(i)(10), Vector3d::UnitY()) *
                //                               AngleAxisd(relative_hand_object_rotation_.col(i)(11), Vector3d::UnitX())).toRotationMatrix();

                Vector3d euler_angles = (hand_rot_curr * relative_hand_object_rotation_).eulerAngles(2, 1, 0);
                object_3d_pose_perturbed_.segment(9, 3) = euler_angles;
            }

            // Evaluate current bounding boxes
            bool valid_bbox;
            MatrixXd curr_bbox(4, pred_bbox_.components);
            std::tie(valid_bbox, curr_bbox) = updateObjectBoundingBox();
            if (valid_bbox)
            {
                // Update change in the bounding box
                delta = curr_bbox - proj_bbox_;

                // Update for next iteration
                proj_bbox_ = curr_bbox;
            }
        // }
    }
    else
    {
        // Initialize for the first time the object pose that
        // will be iteratively updated using only the forward kinematics of the hand
        if (is_object_pose_initialized_)
        {
            // find particles with maximum weight
            int max_index;
            object_3d_pose_weights_.maxCoeff(&max_index);

            object_3d_pose_perturbed_ = object_3d_pose_.col(max_index);
            bool valid_bbox;
            std::tie(valid_bbox, proj_bbox_) = BoundingBoxEstimator::updateObjectBoundingBox();

            if (valid_bbox)
            {
                // Initialization completed
                is_hand_exogenous_initialized_ = true;
            }

            // Store the relative orientation between the hand and the object
            // relative_hand_object_rotation_.resize(3, pred_bbox_.components);

            Matrix3d hand_rot_curr = AngleAxisd(curr_hand_pose(6), curr_hand_pose.segment(3, 3)).toRotationMatrix();

            for (std::size_t i = 0; i < pred_bbox_.components; i++)
            {
                Matrix3d object_rot_curr = (AngleAxisd(object_3d_pose_perturbed_(9), Vector3d::UnitZ()) *
                                            AngleAxisd(object_3d_pose_perturbed_(10), Vector3d::UnitY()) *
                                            AngleAxisd(object_3d_pose_perturbed_(11), Vector3d::UnitX())).toRotationMatrix();

                relative_hand_object_rotation_ = hand_rot_curr.transpose() * object_rot_curr;
            }
        }
    }

    // Update for next iteration
    hand_pose_stamp_ = curr_stamp;
    hand_pose_ = curr_hand_pose;

    return delta;
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
                                bbox(2) = (bottom_right.first - top_left.first) * IOL_bbox_scale_;
                                bbox(3) = (bottom_right.second - top_left.second) * IOL_bbox_scale_;

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


void BoundingBoxEstimator::sendObjectMask()
{
    // Prepare output image
    ImageOf<PixelRgb>& image_out_yarp = port_mask_image_out_.prepare();
    image_out_yarp.resize(object_mask_.cols, object_mask_.rows);
    cv::Mat image_out = yarp::cv::toCvMat(image_out_yarp);

    // Copy mask to the output image
    cv::cvtColor(object_mask_, image_out, CV_GRAY2RGB);

    // Send the image
    port_mask_image_out_.write();
}


std::pair<bool, Eigen::MatrixXd> BoundingBoxEstimator::updateObjectBoundingBox()
{
    // Get camera pose
    yarp::sig::Vector eye_pos_left;
    yarp::sig::Vector eye_att_left;
    yarp::sig::Vector eye_pos_right;
    yarp::sig::Vector eye_att_right;
    if (!gaze_.getCameraPoses(eye_pos_left, eye_att_left, eye_pos_right, eye_att_right))
    {
        // Not updating the bounding box since the camera pose is not available
        return std::make_pair(false, MatrixXd::Zero(4, pred_bbox_.components));
    }

    VectorXd eye_pos;
    VectorXd eye_att;
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

    // Populate particles positions
    std::vector<Superimpose::ModelPoseContainer> si_object_poses(pred_bbox_.components);
    for (std::size_t i = 0; i < pred_bbox_.components; i++)
    {
        // Convert state to Superimpose::ModelPose format
        Superimpose::ModelPose si_object_pose;
        si_object_pose.resize(7);

        // Cartesian coordinates
        si_object_pose[0] = object_3d_pose_perturbed_(0);
        si_object_pose[1] = object_3d_pose_perturbed_(1);
        si_object_pose[2] = object_3d_pose_perturbed_(2);

        // Convert from Euler ZYX to axis/angle
        AngleAxisd angle_axis(AngleAxisd(object_3d_pose_perturbed_(9), Vector3d::UnitZ()) *
                              AngleAxisd(object_3d_pose_perturbed_(10), Vector3d::UnitY()) *
                              AngleAxisd(object_3d_pose_perturbed_(11), Vector3d::UnitX()));
        si_object_pose[3] = angle_axis.axis()(0);
        si_object_pose[4] = angle_axis.axis()(1);
        si_object_pose[5] = angle_axis.axis()(2);
        si_object_pose[6] = angle_axis.angle();

        // Set pose
        Superimpose::ModelPoseContainer& object_pose_container = si_object_poses.at(i);
        object_pose_container.emplace("object", si_object_pose);
    }

    // Project mesh onto the camera plane
    // The shader is designed to have the mesh rendered as a white surface project onto the camera plane
    bool valid_superimpose = false;
    valid_superimpose = object_sicad_->superimpose(si_object_poses, eye_pos.data(), eye_att.data(), object_mask_);

    if (!valid_superimpose)
    {
        std::string err = "BOUNDINGBOXESTIMATOR::UPDATEOBJECTMASK::ERROR\n\tError: cannot superimpose mesh onto the camera plane.";
        throw(std::runtime_error(err));
    }

    // Convert to gray scale
    cv::cvtColor(object_mask_, object_mask_, CV_BGR2GRAY);

    // Send mask for inspection if required
    if (send_mask_)
        sendObjectMask();

    // Evaluate bounding boxes
    MatrixXd curr_bbox(4, pred_bbox_.components);
    for (std::size_t i = 0; i < pred_bbox_.components; i++)
    {
        // Extract the tile relative to the i-th component
        int index_u = i % object_sicad_->getTilesCols();
        int index_v = i / object_sicad_->getTilesCols();

        cv::Rect crop(cv::Point(index_u * cam_width_, index_v * cam_height_),
                      cv::Point((index_u + 1) * cam_width_, (index_v + 1) * cam_height_));
        cv::Mat object_mask_i = object_mask_(crop);

        // Find projected bounding box
        cv::Mat points;
        cv::findNonZero(object_mask_i, points);
        cv::Rect rect = cv::boundingRect(points);

        // Form vector containing center, width and height of projected bounding box
        curr_bbox(0, i) = rect.x + rect.width / 2.0;
        curr_bbox(1, i) = rect.y + rect.height / 2.0;
        curr_bbox(2, i) = rect.width;
        curr_bbox(3, i) = rect.height;
    }

    return std::make_pair(true, curr_bbox);
}

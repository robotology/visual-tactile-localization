#include <BoundingBoxEstimator.h>

#include <yarp/cv/Cv.h>
#include <yarp/sig/Vector.h>

#include <iostream>

using namespace bfl;
using namespace Eigen;
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
    const bool send_mask,
    const Eigen::Ref<const Eigen::MatrixXd>& initial_covariance,
    const Eigen::Ref<const Eigen::MatrixXd>& process_noise_covariance,
    const Eigen::Ref<const Eigen::MatrixXd>& measurement_noise_covariance
) :
    pred_bbox_(number_components, 4),
    corr_bbox_(number_components, 4),
    eye_name_(eye_name),
    IOL_object_name_(IOL_object_name),
    send_mask_(send_mask),
    user_provided_mean_0_(false),
    extractor_(4),
    is_initialized_(false),
    is_exogenous_initialized_(false),
    is_proj_bbox_initialized_(false),
    is_object_pose_initialized_(false),
    gaze_(port_prefix),
    cov_0_(initial_covariance),
    Q_(process_noise_covariance),
    R_(measurement_noise_covariance),
    bbox_width_ratio_(number_components),
    bbox_height_ratio_(number_components)
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
        corr_bbox_.covariance() = cov_0_;

    steady_state_counter_ = 0;
    steady_state_threshold_ = 75;

    // Initialize estimate extractor
    extractor_.setMethod(EstimatesExtraction::ExtractionMethod::emode);
    extractor_.setMobileAverageWindowSize(10);
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
    VectorXd estimate;
    std::tie(std::ignore, estimate) = extractor_.extract(corr_bbox_.mean(), weights);

    return estimate;
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
        for (std::size_t i = 0; i < corr_bbox_.components; i++)
            corr_bbox_.mean(i) = mean_0_;

        is_initialized_ = true;
    }

    for (std::size_t i = 0; i < corr_bbox_.components; i++)
        corr_bbox_.covariance() = cov_0_;

    extractor_.clear();

    steady_state_counter_ = 0;
}


void BoundingBoxEstimator::setObjectPose(const Eigen::Ref<const Eigen::MatrixXd>& pose)
{
    object_3d_pose_ = pose;

    is_object_pose_initialized_ = true;
}


std::size_t BoundingBoxEstimator::getNumberComponents()
{
    return number_components_;
}


void BoundingBoxEstimator::predict()
{
    // state transition
    pred_bbox_.mean() = corr_bbox_.mean();
    pred_bbox_.mean().topRows<2>() += evalExogenousInput().topRows<2>();

    // covariance transition
    for (std::size_t i = 0; i < pred_bbox_.components; i++)
        pred_bbox_.covariance(i) += Q_;
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

    for (std::size_t i = 0; i < pred_bbox_.components; i++)
    {
        MatrixXd Py = pred_bbox_.covariance(i) + R_;

        MatrixXd K = pred_bbox_.covariance(i) * Py.inverse();

        corr_bbox_.mean(i) = pred_bbox_.mean(i) + K * (measured_bbox - pred_bbox_.mean(i));

        corr_bbox_.covariance(i) = pred_bbox_.covariance(i) - K * Py * K.transpose();
    }
}


MatrixXd BoundingBoxEstimator::evalExogenousInput()
{
    if (!updateObjectMask())
        return MatrixXd::Zero(4, pred_bbox_.components);

    MatrixXd exog_input = MatrixXd::Zero(4, pred_bbox_.components);
    MatrixXd curr_bbox(4, pred_bbox_.components);
    std::vector<cv::Rect> bbox_rects(pred_bbox_.components);

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

        bbox_rects.at(i) = rect;
    }

    if (is_proj_bbox_initialized_)
    {
        // Evaluate finite differences
        MatrixXd delta = curr_bbox - proj_bbox_;

        if (!is_exogenous_initialized_)
        {
            if (steady_state_counter_ > steady_state_threshold_)
            {
                is_exogenous_initialized_ = true;

                // Evaluate width and height ratio
                for (std::size_t i = 0; i < pred_bbox_.components; i++)
                {
                    bbox_width_ratio_(i) = corr_bbox_.mean(i, 2) / bbox_rects.at(i).width;
                    bbox_height_ratio_(i) = corr_bbox_.mean(i, 3) / bbox_rects.at(i).height;
                }
            }
        }
        else
        {
            exog_input = delta;
            for (std::size_t i = 0; i < pred_bbox_.components; i++)
            {
                exog_input(2, i) *= bbox_width_ratio_(i);
                exog_input(3, i) *= bbox_height_ratio_(i);
            }
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


bool BoundingBoxEstimator::updateObjectMask()
{
    if (!is_object_pose_initialized_)
        return false;

    std::vector<Superimpose::ModelPoseContainer> si_object_poses(pred_bbox_.components);
    for (std::size_t i = 0; i < pred_bbox_.components; i++)
    {
        // Convert state to Superimpose::ModelPose format
        Superimpose::ModelPose si_object_pose;
        si_object_pose.resize(7);

        // Cartesian coordinates
        si_object_pose[0] = object_3d_pose_(0, i);
        si_object_pose[1] = object_3d_pose_(1, i);
        si_object_pose[2] = object_3d_pose_(2, i);

        // Convert from Euler ZYX to axis/angle
        AngleAxisd angle_axis(AngleAxisd(object_3d_pose_(9, i), Vector3d::UnitZ()) *
                              AngleAxisd(object_3d_pose_(10, i), Vector3d::UnitY()) *
                              AngleAxisd(object_3d_pose_(11, i), Vector3d::UnitX()));
        si_object_pose[3] = angle_axis.axis()(0);
        si_object_pose[4] = angle_axis.axis()(1);
        si_object_pose[5] = angle_axis.axis()(2);
        si_object_pose[6] = angle_axis.angle();

        // Set pose
        Superimpose::ModelPoseContainer& object_pose_container = si_object_poses.at(i);
        object_pose_container.emplace("object", si_object_pose);
    }

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
        valid_superimpose = object_sicad_->superimpose(si_object_poses, eye_pos_left.data(), eye_att_left.data(), object_mask_);
    else if (eye_name_ == "right")
        valid_superimpose = object_sicad_->superimpose(si_object_poses, eye_pos_right.data(), eye_att_right.data(), object_mask_);

    if (!valid_superimpose)
    {
        std::string err = "BOUNDINGBOXESTIMATOR::UPDATEOBJECTMASK::ERROR\n\tError: cannot superimpose mesh onto the camera plane.";
        throw(std::runtime_error(err));
    }

    // Convert to gray scale
    cv::cvtColor(object_mask_, object_mask_, CV_BGR2GRAY);

    if (send_mask_)
        sendObjectMask();

    return true;
}

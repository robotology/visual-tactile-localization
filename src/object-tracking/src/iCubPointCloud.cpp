#include <iCubPointCloud.h>

#include <yarp/cv/Cv.h>
#include <yarp/eigen/Eigen.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>

#include <BayesFilters/Data.h>

#include <Eigen/Dense>

#include <SuperimposeMesh/Superimpose.h>
#include <SuperimposeMesh/SICAD.h>

using namespace bfl;
using namespace yarp::eigen;
using namespace yarp::os;
using namespace yarp::sig;
using namespace Eigen;


iCubPointCloud::iCubPointCloud
(
    std::unique_ptr<PointCloudPrediction> prediction,
    const Ref<const Matrix3d>& noise_covariance_matrix,
    const std::string IOL_object_name,
    const std::string port_prefix,
    const std::string eye_name,
    const std::string obj_mesh_file,
    const std::string sicad_shader_path,
    const std::string depth_fetch_mode,
    const double point_cloud_outlier_threshold,
    const std::size_t point_cloud_u_stride,
    const std::size_t point_cloud_v_stride,
    const bool send_bounding_box,
    const bool send_mask,
    std::shared_ptr<iCubPointCloudExogenousData> exogenous_data
) :
    iCubPointCloud
    (
        std::move(prediction),
        noise_covariance_matrix,
        port_prefix,
        eye_name,
        obj_mesh_file,
        sicad_shader_path,
        depth_fetch_mode,
        point_cloud_outlier_threshold,
        point_cloud_u_stride,
        point_cloud_v_stride,
        send_bounding_box,
        send_mask,
        exogenous_data
    )
{
    IOL_object_name_ = IOL_object_name;

    use_initial_bbox_ = false;

    obj_bbox_set_ = false;
}

iCubPointCloud::iCubPointCloud
(
    std::unique_ptr<PointCloudPrediction> prediction,
    const Ref<const Matrix3d>& noise_covariance_matrix,
    const std::pair<std::pair<int, int>, std::pair<int, int>> initial_bbox,
    const std::string port_prefix,
    const std::string eye_name,
    const std::string obj_mesh_file,
    const std::string sicad_shader_path,
    const std::string depth_fetch_mode,
    const double point_cloud_outlier_threshold,
    const std::size_t point_cloud_u_stride,
    const std::size_t point_cloud_v_stride,
    const bool send_bounding_box,
    const bool send_mask,
    std::shared_ptr<iCubPointCloudExogenousData> exogenous_data

) :
    iCubPointCloud
    (
        std::move(prediction),
        noise_covariance_matrix,
        port_prefix,
        eye_name,
        obj_mesh_file,
        sicad_shader_path,
        depth_fetch_mode,
        point_cloud_outlier_threshold,
        point_cloud_u_stride,
        point_cloud_v_stride,
        send_bounding_box,
        send_mask,
        exogenous_data
    )
{
    // Initialize center, width and height
    auto top_left = initial_bbox.first;
    auto bottom_right = initial_bbox.second;
    obj_bbox_0_.resize(4);
    obj_bbox_0_(0) = (top_left.first + bottom_right.first) / 2.0;
    obj_bbox_0_(1) = (top_left.second + bottom_right.second) / 2.0;
    obj_bbox_0_(2) = (bottom_right.first - top_left.first);
    obj_bbox_0_(3) = (bottom_right.second - top_left.second);

    // Initialize bounding box
    obj_bbox_ = obj_bbox_0_;

    // Remember that the user provided an initial condition within the ctor
    use_initial_bbox_ = true;

    // Set flag
    obj_bbox_set_ = true;
}

iCubPointCloud::iCubPointCloud
(
    std::unique_ptr<PointCloudPrediction> prediction,
    const Eigen::Ref<const Eigen::Matrix3d>& noise_covariance_matrix,
    const std::string port_prefix,
    const std::string eye_name,
    const std::string obj_mesh_file,
    const std::string sicad_shader_path,
    const std::string depth_fetch_mode,
    const double point_cloud_outlier_threshold,
    const std::size_t point_cloud_u_stride,
    const std::size_t point_cloud_v_stride,
    const bool send_bounding_box,
    const bool send_mask,
    std::shared_ptr<iCubPointCloudExogenousData> exogenous_data
) :
    PointCloudModel(std::move(prediction), noise_covariance_matrix),
    eye_name_(eye_name),
    depth_fetch_mode_(depth_fetch_mode),
    pc_outlier_threshold_(point_cloud_outlier_threshold),
    send_bbox_(send_bounding_box),
    send_mask_(send_mask),
    exogenous_data_(exogenous_data),
    gaze_(port_prefix),
    pc_u_stride_(point_cloud_u_stride),
    pc_v_stride_(point_cloud_v_stride)
{
    // Open ports.
    if (!(opc_rpc_client_.open("/" + port_prefix + "/opc/rpc:o")))
    {
        std::string err = "ICUBPOINTCLOUD::CTOR::ERROR\n\tError: cannot open OPC rpc port.";
        throw(std::runtime_error(err));
    }

    if (!(port_depth_in_.open("/" + port_prefix + "/depth:i")))
    {
        std::string err = "ICUBPOINTCLOUD::CTOR::ERROR\n\tError: cannot open depth input port.";
        throw(std::runtime_error(err));
    }

    if (send_bbox_ || send_mask_)
    {
        // Open camera input port.
        if(!port_image_in_.open("/" + port_prefix + "/cam/" + eye_name + ":i"))
        {
            std::string err = "VIEWER::CTOR::ERROR\n\tError: cannot open " + eye_name + " camera input port.";
            throw(std::runtime_error(err));
        }
    }

    if (send_bbox_)
    {
        // Open image output port.
        if(!port_bbox_image_out_.open("/" + port_prefix + "/bbox:o"))
        {
            std::string err = "VIEWER::CTOR::ERROR\n\tError: cannot open bounding box output port.";
            throw(std::runtime_error(err));
        }
    }

    if (send_mask_)
    {
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

    // Configure the default deprojection matrix
    setDefaultDeprojectionMatrix();

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

    // Reset the steady state counter
    steady_state_counter_ = 0;
}


iCubPointCloud::~iCubPointCloud()
{
    // Close ports
    opc_rpc_client_.close();

    if (send_bbox_)
    {
        port_image_in_.close();
        port_bbox_image_out_.close();
    }
}


std::tuple<bool, VectorXd> iCubPointCloud::retrieveObjectBoundingBox(const std::string obj_name)
{
    /*
     * Get object bounding box from OPC module given object name.
     *
     * Adapted from https://github.com/robotology/point-cloud-read
     */
    VectorXd bbox(4);
    bool outcome = false;

    // Command message format is: [ask] (("prop0" "<" <val0>) || ("prop1" ">=" <val1>) ...)
    Bottle cmd, reply;
    cmd.addVocab(Vocab::encode("ask"));
    Bottle &content = cmd.addList().addList();
    content.addString("name");
    content.addString("==");
    content.addString(obj_name);
    opc_rpc_client_.write(cmd,reply);

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

                                outcome = true;
                            }
                        }
                    }
                }
            }
        }
    }

    return std::make_tuple(outcome, bbox);
}


void iCubPointCloud::sendObjectMask(ImageOf<PixelRgb>& camera_image)
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


void iCubPointCloud::sendObjectBoundingBox(ImageOf<PixelRgb>& camera_image)
{
    // Prepare output image
    ImageOf<PixelRgb>& image_out = port_bbox_image_out_.prepare();

    // Copy input to output and wrap around a cv::Mat
    image_out = camera_image;
    cv::Mat image = yarp::cv::toCvMat(image_out);
    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

    // Draw the bounding box
    cv::Point p_tl;
    cv::Point p_br;
    p_tl.x = obj_bbox_(0) - obj_bbox_(2) / 2.0;
    p_tl.y = obj_bbox_(1) - obj_bbox_(3) / 2.0;
    p_br.x = obj_bbox_(0) + obj_bbox_(2) / 2.0;
    p_br.y = obj_bbox_(1) + obj_bbox_(3) / 2.0;

    cv::rectangle(image, p_tl, p_br, cv::Scalar(255, 0, 0));

    // Send the image
    port_bbox_image_out_.write();
}


std::pair<bool, std::vector<std::pair<int, int>>> iCubPointCloud::getObject2DCoordinates(std::size_t stride_u, std::size_t stride_v)
{
    bool valid_coordinates = false;
    std::vector<std::pair<int, int>> coordinates;

    if (!obj_bbox_set_)
        return std::make_pair(false, coordinates);

    std::pair<int, int> top_left;
    top_left.first = int(obj_bbox_(0) - obj_bbox_(2) / 2.0);
    top_left.second = int(obj_bbox_(1) - obj_bbox_(3) / 2.0);

    std::pair<int, int> bottom_right;
    bottom_right.first = int(obj_bbox_(0) + obj_bbox_(2) / 2.0);
    bottom_right.second = int(obj_bbox_(1) + obj_bbox_(3) / 2.0);

    for (std::size_t u = top_left.first; u < bottom_right.first; u += stride_u)
    {
        for (std::size_t v = top_left.second; v < bottom_right.second; v+= stride_v)
        {
            // TODO
            // check for undesired object coordinates
            coordinates.push_back(std::make_pair(u, v));
        }
    }

    return std::make_pair(true, coordinates);
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


bool iCubPointCloud::updateObjectMask()
{
    VectorXd object_pose;
    bool valid_object_pose;
    std::tie(valid_object_pose, object_pose) = exogenous_data_->getObjectEstimate();
    if (!valid_object_pose)
        return false;

    // Convert state to Superimpose::ModelPose format
    Superimpose::ModelPose si_object_pose;
    si_object_pose.resize(7);

    // Cartesian coordinates
    si_object_pose[0] = object_pose(0);
    si_object_pose[1] = object_pose(1);
    si_object_pose[2] = object_pose(2);

    // Convert from Euler ZYX to axis/angle
    AngleAxisd angle_axis(AngleAxisd(object_pose(9), Vector3d::UnitZ()) *
                          AngleAxisd(object_pose(10), Vector3d::UnitY()) *
                          AngleAxisd(object_pose(11), Vector3d::UnitX()));
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
    if (eye_name_ == "left")
        object_sicad_->superimpose(object_pose_container, eye_pos_left.data(), eye_att_left.data(), object_mask_);
    else if (eye_name_ == "right")
        object_sicad_->superimpose(object_pose_container, eye_pos_right.data(), eye_att_right.data(), object_mask_);

    // Convert to gray scale
    cv::cvtColor(object_mask_, object_mask_, CV_BGR2GRAY);

    return true;
}

void iCubPointCloud::updateObjectBoundingBox()
{
    if (steady_state_counter_ < steady_state_thr_)
        return;

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

    if (steady_state_counter_ == steady_state_thr_)
    {
        // Evaluate width and height ratio
        bbox_width_ratio_ = obj_bbox_(2) / bbox_rect.width;
        bbox_height_ratio_ = obj_bbox_(3) / bbox_rect.height;
    }
    else
    {
        // Evaluate finite differences
        VectorXd delta = curr_bbox - proj_bbox_;
        delta(2) *= bbox_width_ratio_;
        delta(3) *= bbox_height_ratio_;

        // Predict new bounding box
        obj_bbox_ += delta;
    }

    // Store projected bounding box for next iteration
    proj_bbox_ = curr_bbox;
}


void iCubPointCloud::reset()
{
    // By resetting this, the bounding box is initialized again using OPC
    // when freezeMeasurements is called next time
    obj_bbox_set_ = false;

    // However if the user provided an initial bounding box
    // then it is required to set it again
    if (use_initial_bbox_)
    {
        obj_bbox_ = obj_bbox_0_;

        obj_bbox_set_ = true;
    }

    // Reset the exogenous data class
    exogenous_data_->reset();

    // Reset the steady state counter
    steady_state_counter_ = 0;
}


bool iCubPointCloud::freezeMeasurements()
{
    // HINT: maybe a blocking style may be better, i.e., while (!object_bbox_set_).
    if (!obj_bbox_set_)
    {
        std::tie(obj_bbox_set_, obj_bbox_) = retrieveObjectBoundingBox(IOL_object_name_);

        if (!obj_bbox_set_)
        {
            // Return false. Next call to freezeMeasurements will try to get the bounding box.
            return false;
        }
    }

    // Update object mask
    updateObjectMask();

    // Update bounding box
    updateObjectBoundingBox();

    // Send bounding box and mask over the network
    ImageOf<PixelRgb>* image_in;
    if (send_bbox_ || send_mask_)
         image_in = port_image_in_.read(false);

    if (image_in != nullptr)
    {
        if (send_bbox_)
            sendObjectBoundingBox(*image_in);

        if (send_mask_)
            sendObjectMask(*image_in);
    }

    // Get 2d coordinates
    bool valid_coordinates;
    std::vector<std::pair<int, int>> coordinates;

    std::tie(valid_coordinates, coordinates) = getObject2DCoordinates(pc_u_stride_, pc_v_stride_);
    if (!valid_coordinates)
        return false;

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

    // take only valid points
    MatrixXd points(3, good_points.sum());
    for (int i = 0, j = 0; i < point_cloud.cols(); i++)
    {
        if (good_points(i) == 1)
        {
            point_cloud.col(i).swap(points.col(j));
            j++;
        }
    }

    // Resize measurements to be a column vector.
    measurement_.resize(3 * points.cols(), 1);
    measurement_.swap(Map<MatrixXd>(points.data(), points.size(), 1));

    logger(measurement_.transpose());

    // WARNING: this has to be removed at some point
    steady_state_counter_++;

    return true;
}


std::pair<bool, Data> iCubPointCloud::measure() const
{
    return std::make_pair(true, measurement_);
}


std::pair<std::size_t, std::size_t> iCubPointCloud::getOutputSize() const
{
    return std::make_pair(measurement_.size(), 0);
}


bool iCubPointCloud::setProperty(const std::string& property)
{
    bool set_successful = false;

    if (property == "reset")
    {
        reset();
        set_successful = true;
    }

    return set_successful;
}


iCubPointCloudExogenousData::iCubPointCloudExogenousData() :
    obj_estimate_set_(false)
{ }


iCubPointCloudExogenousData:: ~iCubPointCloudExogenousData()
{ }


void iCubPointCloudExogenousData::setObjectEstimate(const Ref<const VectorXd>& pose)
{
    last_estimate_ = pose;

    obj_estimate_set_ = true;
}


std::pair<bool, VectorXd> iCubPointCloudExogenousData::getObjectEstimate()
{
    return std::make_pair(obj_estimate_set_, last_estimate_);
}


void iCubPointCloudExogenousData::reset()
{
    obj_estimate_set_ = false;
}

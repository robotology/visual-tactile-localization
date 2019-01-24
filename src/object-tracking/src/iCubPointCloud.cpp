#include <iCubPointCloud.h>

#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>

#include <BayesFilters/Data.h>

#include <Eigen/Dense>

#include <SuperimposeMesh/Superimpose.h>
#include <SuperimposeMesh/SICAD.h>

using namespace bfl;
using namespace yarp::os;
using namespace Eigen;


iCubPointCloud::iCubPointCloud
(
    const string port_prefix,
    const string SFM_context_name,
    const string SFM_config_name,
    const string IOL_object_name,
    const string obj_mesh_file,
    const string sicad_shader_path,
    const string eye_name,
    std::unique_ptr<PointCloudPrediction> prediction,
    const Eigen::Ref<const Eigen::Matrix3d>& noise_covariance_matrix,
    std::shared_ptr<iCubPointCloudExogenousData> exogenous_data
) :
    iCubPointCloud
    (
        port_prefix,
        SFM_context_name,
        SFM_config_name,
        obj_mesh_file,
        sicad_shader_path,
        eye_name,
        std::move(prediction),
        noise_covariance_matrix,
        exogenous_data
    )
{
    IOL_object_name_ = IOL_object_name;

    obj_bbox_set_ = false;
}

iCubPointCloud::iCubPointCloud
(
    const string port_prefix,
    const string SFM_context_name,
    const string SFM_config_name,
    const string obj_mesh_file,
    const string sicad_shader_path,
    const string eye_name,
    const std::pair<std::pair<int, int>, std::pair<int, int>> initial_bbox,
    std::unique_ptr<PointCloudPrediction> prediction,
    const Eigen::Ref<const Eigen::Matrix3d>& noise_covariance_matrix,
    std::shared_ptr<iCubPointCloudExogenousData> exogenous_data
) :
    iCubPointCloud
    (
        port_prefix,
        SFM_context_name,
        SFM_config_name,
        obj_mesh_file,
        sicad_shader_path,
        eye_name,
        std::move(prediction),
        noise_covariance_matrix,
        exogenous_data
    )
{
    obj_bbox_tl_ = initial_bbox.first;
    obj_bbox_br_ = initial_bbox.second;

    obj_bbox_set_ = true;
}

iCubPointCloud::iCubPointCloud
(
    const string port_prefix,
    const string SFM_context_name,
    const string SFM_config_name,
    const string obj_mesh_file,
    const string sicad_shader_path,
    const string eye_name,
    std::unique_ptr<PointCloudPrediction> prediction,
    const Eigen::Ref<const Eigen::Matrix3d>& noise_covariance_matrix,
    std::shared_ptr<iCubPointCloudExogenousData> exogenous_data
) :
    PointCloudModel(std::move(prediction), noise_covariance_matrix),
    gaze_(port_prefix),
    eye_name_(eye_name),
    obj_bbox_estimator_(4),
    exogenous_data_(exogenous_data),
    sfm_(port_prefix)
{
    // Open ports.
    if (!(opc_rpc_client_.open("/" + port_prefix + "/opc/rpc:o")))
    {
        std::string err = "ICUBPOINTCLOUD::CTOR::ERROR\n\tError: cannot open OPC rpc port.";
        throw(std::runtime_error(err));
    }

    // Configure SFM library.
    ResourceFinder rf_sfm;
    rf_sfm.setVerbose(true);
    rf_sfm.setDefaultConfigFile(SFM_config_name.c_str());
    rf_sfm.setDefaultContext(SFM_context_name.c_str());
    rf_sfm.configure(0, NULL);

    if (!sfm_.configure(rf_sfm, port_prefix))
    {
        std::string err = "ICUBPOINTCLOUD::CTOR::ERROR\n\tError: cannot configure instance of SFM library.";
        throw(std::runtime_error(err));
    }

    // Get iCub cameras intrinsics parameters
    double cam_fx;
    double cam_fy;
    double cam_cx;
    double cam_cy;
    if (!gaze_.getCameraIntrinsics(eye_name, cam_fx, cam_fy, cam_cx, cam_cy))
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
                      cam_fx,
                      cam_fy,
                      cam_cx,
                      cam_cy,
                      1,
                      sicad_shader_path,
                      {1.0, 0.0, 0.0, static_cast<float>(M_PI)})
        );

    // Configure object bounding box estimator to use exponential mean
    obj_bbox_estimator_.setMethod(EstimatesExtraction::ExtractionMethod::emean);
    // Leave default window size
    // obj_bbox_estimator_.setMobileAverageWindowSize();
}


iCubPointCloud::~iCubPointCloud()
{
    // Close ports
    opc_rpc_client_.close();
}


std::tuple<bool, std::pair<int, int>, std::pair<int, int>> iCubPointCloud::retrieveObjectBoundingBox(const string obj_name)
{
    /*
     * Get object bounding box from OPC module given object name.
     *
     * Adapted from https://github.com/robotology/point-cloud-read
     */

    std::pair<int, int> top_left;
    std::pair<int, int> bottom_right;

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
                                //  position_2d_left contains x,y of top left and x,y of bottom right
                                top_left.first      = position_2d_bb->get(0).asInt();
                                top_left.second     = position_2d_bb->get(1).asInt();
                                bottom_right.first  = position_2d_bb->get(2).asInt();
                                bottom_right.second = position_2d_bb->get(3).asInt();

                                outcome = true;
                            }
                        }
                    }
                }
            }
        }
    }

    return std::make_tuple(outcome, top_left, bottom_right);
}


std::pair<bool, std::vector<std::pair<int, int>>> iCubPointCloud::getObject2DCoordinates(std::size_t stride_u, std::size_t stride_v)
{
    bool valid_coordinates = false;
    std::vector<std::pair<int, int>> coordinates;

    if (!obj_bbox_set_)
        return std::make_pair(false, coordinates);

    for (std::size_t u = obj_bbox_tl_.first; u < obj_bbox_br_.first; u += stride_u)
    {
        for (std::size_t v = obj_bbox_tl_.second; v < obj_bbox_br_.second; v+= stride_v)
        {
            // TODO
            // check for undesired object coordinates
            coordinates.push_back(std::make_pair(u, v));
        }
    }

    return std::make_pair(true, coordinates);
}


void iCubPointCloud::updateObjectBoundingBox(const Ref<const VectorXd>& object_pose)
{
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
        return;
    }

    // Project mesh onto the camera plane
    // The shader is designed to have the mesh rendered as a white surface project onto the camera plane
    cv::Mat rendered_image;
    if (eye_name_ == "left")
        object_sicad_->superimpose(object_pose_container, eye_pos_left.data(), eye_att_left.data(), rendered_image);
    else if (eye_name_ == "right")
        object_sicad_->superimpose(object_pose_container, eye_pos_right.data(), eye_att_right.data(), rendered_image);

    // Find bounding box
    cv::Mat points;
    cv::findNonZero(rendered_image, points);
    cv::Rect bbox_rect = cv::boundingRect(points);
    VectorXd bbox_coordinates(4);
    bbox_coordinates(0) = bbox_rect.x;
    bbox_coordinates(1) = bbox_rect.y;
    bbox_coordinates(2) = bbox_rect.x + bbox_rect.width;
    bbox_coordinates(3) = bbox_rect.y + bbox_rect.height;
    // EstimatesExtractor requires weights in logspace
    // Since we have only a sample, only one weight is required
    VectorXd weight(1);
    weight(0) = std::log(1);
    VectorXd filtered_bbox;
    std::tie(std::ignore, filtered_bbox) = obj_bbox_estimator_.extract(bbox_coordinates, weight);

    // Update bounding box
    obj_bbox_tl_.first = filtered_bbox(0);
    obj_bbox_tl_.second = filtered_bbox(1);
    obj_bbox_br_.first = filtered_bbox(2);
    obj_bbox_br_.second = filtered_bbox(3);
}


void iCubPointCloud::reset()
{
    // By resetting this, the bounding box is initialized again using OPC
    // when freezeMeasurements is called next time
    obj_bbox_set_ = false;

    // Reset the exogenous data class
    exogenous_data_->reset();
}


bool iCubPointCloud::freezeMeasurements()
{
    // HINT: maybe a blocking style may be better, i.e., while (!object_bbox_set_).
    if (!obj_bbox_set_)
    {
        std::tie(obj_bbox_set_, obj_bbox_tl_, obj_bbox_br_) = retrieveObjectBoundingBox(IOL_object_name_);

        if (!obj_bbox_set_)
        {
            // Return false. Next call to freezeMeasurements will try to get the bounding box.
            return false;
        }
    }

    // Update bounding box using last state estimate
    // provided by exogenous data
    VectorXd last_estimate;
    bool valid_last_estimate;
    std::tie(valid_last_estimate, last_estimate) = exogenous_data_->getObjectEstimate();
    if (valid_last_estimate)
        updateObjectBoundingBox(last_estimate);

    // Get 2D coordinates.
    std::size_t stride_u = 1;
    std::size_t stride_v = 1;

    bool valid_coordinates;
    std::vector<std::pair<int, int>> coordinates;

    std::tie(valid_coordinates, coordinates) = getObject2DCoordinates(stride_u, stride_v);
    if (!valid_coordinates)
        return false;

    // Get 3D point cloud.
    bool blocking_call = false;
    bool valid_point_cloud;
    MatrixXd point_cloud;
    std::tie(valid_point_cloud, point_cloud) = sfm_.get3DPoints(coordinates, blocking_call);
    if (!valid_point_cloud)
        return false;

    // Resize measurements to be a column vector.
    measurement_.resize(3 * point_cloud.cols(), 1);
    measurement_.swap(Map<MatrixXd>(point_cloud.data(), point_cloud.size(), 1));

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

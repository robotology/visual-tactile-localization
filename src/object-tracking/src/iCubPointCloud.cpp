#include <iCubPointCloud.h>

#include <yarp/os/ResourceFinder.h>

#include <BayesFilters/Data.h>

#include <Eigen/Dense>

using namespace bfl;
using namespace yarp::os;
using namespace Eigen;

iCubPointCloud::iCubPointCloud
(
    const string port_prefix,
    const string SFM_context_name,
    const string SFM_config_name,
    const string IOL_object_name,
    std::unique_ptr<PointCloudPrediction> prediction,
    const Eigen::Ref<const Eigen::Matrix3d>& noise_covariance_matrix
) :
    PointCloudModel(std::move(prediction), noise_covariance_matrix),
    IOL_object_name_(IOL_object_name)
{
    // Open ports.
    opc_rpc_client_.open("/" + port_prefix + "/opc/rpc:o");

    // Configure SFM library.
    ResourceFinder rf_sfm;
    rf_sfm.setVerbose(true);
    rf_sfm.setDefaultConfigFile(SFM_config_name.c_str());
    rf_sfm.setDefaultContext(SFM_context_name.c_str());
    rf_sfm.configure(0, NULL);

    sfm_.configure(rf_sfm, port_prefix);

    // Reset flags
    obj_bbox_set_ = false;
    obj_estimate_set_ = false;
    ext_obj_bbox_set_ = false;
}


iCubPointCloud::~iCubPointCloud()
{
    // Close ports
    opc_rpc_client_.close();
}


void iCubPointCloud::setExternalObjectBoundingBox(std::pair<int, int> top_left, std::pair<int, int> bottom_right)
{
    ext_obj_bbox_tl_ = top_left;
    ext_obj_bbox_br_ = bottom_right;

    ext_obj_bbox_set_ = true;
}


void iCubPointCloud::setObjectEstimate(Ref<const VectorXd>& pose)
{
    last_estimate_ = pose;

    obj_estimate_set_ = true;
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
            // check for undesired object coordinates
            if (ext_obj_bbox_set_)
            {
                if ((u >= ext_obj_bbox_tl_.first)  && (u <= ext_obj_bbox_br_.first) &&
                    (v >= ext_obj_bbox_tl_.second) && (v <= ext_obj_bbox_br_.second))
                    continue;
            }

            coordinates.push_back(std::make_pair(u, v));
        }
    }

    return std::make_pair(true, coordinates);
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
    measurement_.resize(3 * point_cloud.cols());
    measurement_.swap(Map<VectorXd>(point_cloud.data(), point_cloud.size()));

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

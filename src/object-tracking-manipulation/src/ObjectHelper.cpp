#include <ObjectHelper.h>

#include <iCub/ctrl/math.h>

#include <yarp/eigen/Eigen.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>

#include <Eigen/Dense>

using namespace Eigen;
using namespace iCub::ctrl;
using namespace iCub::iKin;
using namespace yarp::eigen;
using namespace yarp::os;
using namespace yarp::sig;


ObjectHelper::ObjectHelper(const std::string port_prefix, const std::string context, const std::string laterality) :
    icub_arm_(iCubArm(laterality + "_v2"))
{

    if (!port_torso_enc_.open("/" + port_prefix + "/torso:i"))
    {
        throw std::runtime_error("ERROR::" + log_ID_ + "::CTOR::\nERROR:: cannot open torso input port");
    }

    if (!port_arm_enc_.open("/" + port_prefix + "/" + laterality + "_arm:i"))
    {
        throw std::runtime_error("ERROR::" + log_ID_ + "::CTOR::\nERROR:: cannot open arm input port");
    }

    if (!port_object_pose_in_.open("/" + port_prefix + "/object_pose:i"))
    {
        throw std::runtime_error("ERROR::" + log_ID_ + "::CTOR::\nERROR:: cannot open object pose input port");
    }

    if (!port_hand_pose_in_.open("/" + port_prefix + "/hand_pose:i"))
    {
        throw std::runtime_error("ERROR::" + log_ID_ + "::CTOR::\nERROR:: cannot open hand pose input port");
    }

    ResourceFinder rf_object_tracking;
    rf_object_tracking.setVerbose(true);
    rf_object_tracking.setDefaultContext("object-tracking");
    rf_object_tracking.setDefaultConfigFile("config.ini");
    rf_object_tracking.configure(0, NULL);

    ResourceFinder rf_object_name = rf_object_tracking.findNestedResourceFinder("OBJECT");
    object_name_ = rf_object_name.check("object_name", Value("ycb_mustard")).asString();
    yInfo() << log_ID_ << "- object_name:" << object_name_;

    ResourceFinder rf_object_helper;
    rf_object_helper.setVerbose(true);
    rf_object_helper.setDefaultContext("object-tracking-manipulation");
    rf_object_helper.setDefaultConfigFile("object_helper_config.ini");
    rf_object_helper.configure(0, NULL);

    ResourceFinder rf_object = rf_object_helper.findNestedResourceFinder(object_name_.c_str());
    // Load default approaching configuration for the specified object
    bool valid_approach_configuration;
    yarp::sig::Vector approach_configuration;
    std::tie(valid_approach_configuration, approach_configuration) =
        loadVectorDouble(rf_object, laterality + "_approach_pose", 6);

    if (!valid_approach_configuration)
    {
        throw std::runtime_error("ERROR::" + log_ID_ + "::CTOR::\nERROR: cannot load the default approaching configuration.");
    }
    approach_position_.resize(3);
    approach_position_ = approach_configuration.subVector(0, 2);
    approach_orientation_.resize(3);
    approach_orientation_ = approach_configuration.subVector(3, 5);

    // Initialize the iCub forward kinematics in order to consider also the torso
    icub_arm_.setAllConstraints(false);
    icub_arm_.releaseLink(0);
    icub_arm_.releaseLink(1);
    icub_arm_.releaseLink(2);
}


ObjectHelper::~ObjectHelper()
{ }


yarp::sig::Vector ObjectHelper::getCoarseApproachPoint()
{
    updateObjectPose();
    evaluateApproachPosition();

    // This is to be sent to the Cartesian controller without
    // compensation for the mismatch between the stereo vision and
    // the cartesian domain of the robot.
    // Hence, an arbitrary offset is added to this
    yarp::sig::Vector offset(3);
    offset(0) = 0.0;
    offset(1) = 0.1;
    offset(2) = 0.0;

    return approach_position_robot_ + offset;
}


yarp::sig::Vector ObjectHelper::getPreciseApproachPoint()
{
    updateHandForwardKinematics();
    updateHandPose();

    // Suppose that the end effector has been already placed in the
    // position suggested by getCoarseApproachPosition
    // At this point it is known
    // - where the hand is according to the forward kinematics
    // - where the hand is in the stereo vision domain
    // - where the approach point is in the stereo vision domain

    // Evaluate the linear map from the stereo vision domain
    // to the forward kinematic domain

    // First write the homogeneous transform associated to the filtered hand pose
    Transform<double, 3, Affine> root_frame_to_filtered_hand;
    root_frame_to_filtered_hand = Translation<double, 3>(toEigen(hand_pose_.subVector(0, 2)));
    root_frame_to_filtered_hand.rotate(AngleAxisd(hand_pose_(6), toEigen(hand_pose_.subVector(3, 5))));
    root_frame_to_filtered_hand = root_frame_to_filtered_hand.inverse();

    // Then write the homogeneous transform associated to the forward kinematic
    Transform<double, 3, Affine> fk_to_root_frame;
    fk_to_root_frame = Translation<double, 3>(toEigen(hand_fk_.subVector(0, 2)));
    fk_to_root_frame.rotate(AngleAxisd(hand_fk_(6), toEigen(hand_fk_.subVector(3, 5))));

    // Finally evaluate the position in the cartesian domain that should produce
    // a contact between palm and object surface in the *visual domain*
    Vector3d approach_point = fk_to_root_frame * root_frame_to_filtered_hand * toEigen(approach_position_robot_).homogeneous();
    yarp::sig::Vector approach_point_yarp(3, approach_point.data());

    return approach_point_yarp;
}


bool ObjectHelper::updateHandForwardKinematics()
{
    yarp::sig::Vector encoders(10, 0.0);

    Bottle* bottle_torso = port_torso_enc_.read(true);
    Bottle* bottle_arm  = port_arm_enc_.read(true);
    if (!bottle_torso || !bottle_arm)
        return false;

    encoders(0) = bottle_torso->get(2).asDouble();
    encoders(1) = bottle_torso->get(1).asDouble();
    encoders(2) = bottle_torso->get(0).asDouble();

    for (std::size_t i = 0; i < 7; i++)
        encoders(3 + i) = bottle_arm->get(i).asDouble();

    // Update the chain
    icub_arm_.setAng(encoders * CTRL_DEG2RAD);

    // Get the pose in position/axis/angle representation
    hand_fk_ = icub_arm_.EndEffPose();

    return true;
}


bool ObjectHelper::updateHandPose()
{
    yarp::sig::Vector* hand_pose = port_hand_pose_in_.read(true);

    if (hand_pose != nullptr)
        hand_pose_ = *hand_pose;
    else
        return false;

    return true;
}


bool ObjectHelper::updateObjectPose()
{
    yarp::sig::Vector* object_pose = port_object_pose_in_.read(true);

    if (object_pose != nullptr)
        object_pose_ = *object_pose;
    else
        return false;

    return true;
}


void ObjectHelper::evaluateApproachPosition()
{
    // Evaluate the homogeneous transform associated to the object
    Transform<double, 3, Eigen::Affine> object_to_robot;
    object_to_robot = Translation<double, 3>(toEigen(object_pose_.subVector(0, 2)));
    object_to_robot.rotate(AngleAxisd(object_pose_(6), toEigen(object_pose_.subVector(3, 5))));

    // Do object-specific overrides
    if (object_name_ == "ycb_mustard_bottle")
    {
        // Get the center of the mesh in robot root frame
        Vector3d mesh_0 = toEigen(object_pose_.subVector(0, 2));

        // Get the point corresponding to (1, 0, 0) in local frame
        Vector3d x_axis_tip = object_to_robot * Vector3d::UnitX().homogeneous();

        // Evaluate the difference vector
        Vector3d x_axis = x_axis_tip - mesh_0;

        // If the x_axis is pointing towards iCub -y direction,
        // it is required to rotate the pose by 180 degrees about the robot z axis
        if (x_axis(1) < 0)
            object_to_robot.rotate(AngleAxisd(M_PI, Vector3d::UnitZ()));
    }

    // Evaluate the effective approach point
    Vector3d approach_point = object_to_robot * toEigen(approach_position_).homogeneous();
    approach_position_robot_ = yarp::sig::Vector(3, approach_point.data());
}


std::pair<bool, yarp::sig::Vector> ObjectHelper::loadVectorDouble
(
    const ResourceFinder& rf,
    const std::string key,
    const std::size_t size
)
{
    bool ok = true;

    if (rf.find(key).isNull())
        ok = false;

    Bottle* b = rf.find(key).asList();
    if (b == nullptr)
        ok = false;

    if (b->size() != size)
        ok = false;

    if (!ok)
        return std::make_pair(false, yarp::sig::Vector());

    yarp::sig::Vector vector(size);
    for (std::size_t i = 0; i < b->size(); i++)
    {
        Value item_v = b->get(i);
        if (item_v.isNull())
            return std::make_pair(false, yarp::sig::Vector());

        if (!item_v.isDouble())
            return std::make_pair(false, yarp::sig::Vector());

        vector(i) = item_v.asDouble();
    }

    return std::make_pair(true, vector);
}

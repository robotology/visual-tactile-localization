#ifndef OBJECTHELPER_H
#define OBJECTHELPER_H

#include <iCub/iKin/iKinFwd.h>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>


class ObjectHelper
{
public:
    ObjectHelper(const std::string port_prefix, const std::string context, const std::string laterality);

    virtual ~ObjectHelper();

    yarp::sig::Vector getCoarseApproachPoint();

    yarp::sig::Vector getPreciseApproachPoint();

    yarp::sig::Vector getRequiredHandOrientation();

protected:
    bool updateHandForwardKinematics();

    bool updateHandPose();

    bool updateObjectPose();

    void evaluateApproachPosition();

    std::pair<yarp::sig::Vector, yarp::sig::Vector> getApproachConfiguration();

    std::pair<bool, yarp::sig::Vector> loadVectorDouble(const yarp::os::ResourceFinder& rf, const std::string key, const std::size_t size);

    yarp::os::BufferedPort<yarp::sig::Vector> port_object_pose_in_;

    yarp::os::BufferedPort<yarp::sig::Vector> port_hand_pose_in_;

    yarp::os::BufferedPort<yarp::os::Bottle> port_torso_enc_;

    yarp::os::BufferedPort<yarp::os::Bottle> port_arm_enc_;

    const std::string log_ID_ = "[OBJECTHELPER]";

    yarp::sig::Vector approach_position_;

    yarp::sig::Vector approach_position_robot_;

    yarp::sig::Vector approach_orientation_;

    iCub::iKin::iCubArm icub_arm_;

    yarp::sig::Vector hand_pose_;

    yarp::sig::Vector hand_fk_;

    yarp::sig::Vector object_pose_;

    std::string object_name_;
};

#endif

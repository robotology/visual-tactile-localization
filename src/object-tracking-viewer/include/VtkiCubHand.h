#ifndef VTKICUBHAND_H
#define VTKICUBHAND_H

#include <iCubArmModel.h>
#include <VtkMesh.h>

#include <unordered_map>

#include <vtkRenderer.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

class VtkiCubHand
{
public:
    VtkiCubHand(const std::string port_prefix, const std::string laterality);

    virtual ~VtkiCubHand();

    void addToRenderer(vtkRenderer& renderer);

    bool updateHandPose();

private:
    std::unordered_map<std::string, VtkMesh> meshes_;

    yarp::os::BufferedPort<yarp::sig::Vector> hand_pose_port_in;

    iCubArmModel hand_model_;

    const std::string log_ID_ = "[VTKICUBHAND]";
};

#endif /* VTKICUBHAND_H */

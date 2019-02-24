#ifndef ICUBHANDCONTACTSMODEL_H
#define ICUBHANDCONTACTSMODEL_H

#include <ContactDetection.h>
#include <iCubArmModel.h>
#include <MeshImporter.h>
#include <VCGTriMesh.h>

#include <Eigen/Dense>

#include <unordered_map>

#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

class iCubHandContactsModel
{
public:
    iCubHandContactsModel
    (
        std::unique_ptr<iCubArmModel> icub_arm,
        std::unique_ptr<ContactDetection> contact_detection,
        std::vector<std::string> used_fingers,
        const std::string port_prefix
    );

    virtual ~iCubHandContactsModel();

    bool freezeMeasurements();

    void printContactDetection();

    Eigen::VectorXd measure() const;

protected:
    bool isHandPartEnabled(const std::string hand_part_name);

    std::string getFingerTipName(const std::string finger_name);

    void sampleFingerTip(const std::string fingertip_name);

    bool loadMesh(const std::string hand_part_name, const std::string mesh_path);

    yarp::os::BufferedPort<yarp::sig::Vector> hand_pose_port_in;

    std::unordered_map<std::string, simpleTriMesh> hand_meshes_;

    std::unordered_map<std::string, Eigen::MatrixXd> sampled_fingertips_;

    Eigen::VectorXd measurements_;

    std::unordered_map<std::string, Eigen::Map<Eigen::MatrixXd>> measurements_accessor_;

    const std::vector<std::string> used_fingers_;

    std::unique_ptr<iCubArmModel> icub_arm_;

    std::unique_ptr<ContactDetection> contact_detection_;

    const std::string log_ID_ = "[ICUBHANDCONTACTSMODEL]";

    const std::size_t fingertip_number_samples_ = 25;
};

#endif /* ICUBHANDCONTACTSMODEL_H */


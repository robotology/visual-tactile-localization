#ifndef ICUBHANDCONTACTSMODEL_H
#define ICUBHANDCONTACTSMODEL_h

#include <iCubArmModel.h>
#include <MeshImporter.h>
#include <VCGTriMesh.h>

#include <Eigen/Dense>

#include <unordered_map>

class iCubHandContactsModel
{
public:
    iCubHandContactsModel(std::unique_ptr<iCubArmModel> icub_arm, std::vector<std::string> used_fingers);

protected:
    bool isHandPartEnabled(const std::string hand_part_name);

    std::string getFingerTipName(const std::string finger_name);

    void sampleFingerTip(const std::string fingertip_name);

    bool loadMesh(const std::string hand_part_name, const std::string mesh_path);

    std::unordered_map<std::string, simpleTriMesh> hand_meshes_;

    std::unordered_map<std::string, Eigen::VectorXd> sampled_fingertips_;

    const std::vector<std::string> used_fingers_;

    std::unique_ptr<iCubArmModel> icub_arm_;

    const std::string log_ID_ = "[ICUBHANDCONTACTSMODEL]";

    const std::size_t fingertip_number_samples_ = 50;
};

#endif /* ICUBHANDCONTACTSMODEL_H */


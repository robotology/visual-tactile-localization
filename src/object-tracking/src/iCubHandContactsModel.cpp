/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <iCubHandContactsModel.h>

#include <yarp/eigen/Eigen.h>
#include <yarp/os/LogStream.h>

using namespace Eigen;
using namespace yarp::eigen;


iCubHandContactsModel::iCubHandContactsModel
(
    std::unique_ptr<iCubArmModel> icub_arm,
    std::unique_ptr<ContactDetection> contact_detection,
    std::vector<std::string> used_fingers,
    const std::string port_prefix
) :
    icub_arm_(std::move(icub_arm)),
    contact_detection_(std::move(contact_detection)),
    used_fingers_(used_fingers)
{
    // Try to open the hand pose input port
    if (!(hand_pose_port_in.open("/" + port_prefix + "/hand_pose:i")))
    {
        std::string err = "ICUBHANDCONTACTSMODEL::CTOR::ERROR\n\tError: cannot open hand pose input port.";
        throw(std::runtime_error(err));
    }

    // Retrieve icub hand mesh paths
    bool valid_paths;
    SICAD::ModelPathContainer meshes_paths;
    std::tie(valid_paths, meshes_paths) = icub_arm_->getMeshPaths();

    if (!valid_paths)
    {
        std::string err = "ICUBHANDCONTACTSMODEL::CTOR::ERROR\n\tError: cannot load iCub hand meshes paths.";
        throw(std::runtime_error(err));
    }

    // Load all the meshes
    for (auto path : meshes_paths)
    {
        if (isHandPartEnabled(path.first))
        {
            if (!loadMesh(path.first, path.second))
            {
                std::string err = "ICUBHANDCONTACTSMODEL::CTOR::ERROR\n\tError: cannot load iCub hand meshes.";
                throw(std::runtime_error(err));
            }

            yInfo() << log_ID_ << "Mesh of hand part " + path.first + " successfully loaded.";
        }
    }

    // Load also additional meshes of cleaned fingertips
    for (auto finger_name : used_fingers_)
    {
        for (auto path : meshes_paths)
        {
            if(path.first.find(getFingerTipName(finger_name)) != string::npos)
            {
                // removes .obj and append _cleaned.obj
                std::string new_path = path.second.substr(0, path.second.size() - 4) + "_cleaned.obj";

                if (!loadMesh(path.first + "cleaned", new_path))
                {
                    std::string err = "ICUBHANDCONTACTSMODEL::CTOR::ERROR\n\tError: cannot load iCub hand meshes.";
                    throw(std::runtime_error(err));
                }

                yInfo() << log_ID_ << "Mesh of hand part " + path.first + "(cleaned) successfully loaded.";
            }
        }
    }

    // Sample clouds on fingertips
    for (auto used_finger : used_fingers_)
        sampleFingerTip(getFingerTipName(used_finger));

    // Resize the output measurement vector
    std::size_t number_samples = 0;
    for (auto used_finger : used_fingers_)
        number_samples += sampled_fingertips_[getFingerTipName(used_finger)].cols();
    measurements_ = VectorXd::Zero(3 * number_samples);

    // Evaluate accessors to subvector of the measurement vector
    for (std::size_t i = 0, total_size = 0; i < used_fingers_.size(); i++)
    {
        std::string fingertip_name = getFingerTipName(used_fingers_[i]);

        int sample_size = sampled_fingertips_.at(fingertip_name).cols();

        measurements_accessor_.emplace(std::make_pair(fingertip_name, Map<MatrixXd>(measurements_.segment(total_size, sample_size * 3).data(), 3, sample_size)));

        total_size += sample_size * 3;
    }
}


iCubHandContactsModel::~iCubHandContactsModel()
{
    hand_pose_port_in.close();
}


bool iCubHandContactsModel::freezeMeasurements()
{
    // TODO: maybe it is better to store the previous pose
    // and return this in case of missing read

    // Get the hand of the pose
    yarp::sig::Vector* hand_pose_yarp = hand_pose_port_in.read(false);

    if (hand_pose_yarp == nullptr)
        return false;

    VectorXd hand_pose = toEigen(*hand_pose_yarp);

    // Get the pose of all part of the hand according to finger forward kinematics
    bool valid_hand_model_pose;
    std::vector<Superimpose::ModelPoseContainer> vector_poses;
    std::tie(valid_hand_model_pose, vector_poses) = icub_arm_->getModelPose(hand_pose);

    if (!valid_hand_model_pose)
        return false;

    Superimpose::ModelPoseContainer& poses = vector_poses[0];

    // Transform points sampled on the fingertips
    // to the root reference frame of the robot
    for (auto used_finger : used_fingers_)
    {
        std::string fingertip_name = getFingerTipName(used_finger);

        const Superimpose::ModelPose& fingertip_pose = poses.find(fingertip_name)->second;
        Map<const VectorXd> pose_eigen(fingertip_pose.data(), 7);

        // Compose the transformation
        // pose_eigen is expressed as (cartesian, axis, angle)
        Transform<double, 3, Eigen::Affine> finger_to_root;
        finger_to_root = Translation<double, 3>(pose_eigen.segment(0, 3));
        finger_to_root.rotate(AngleAxisd(pose_eigen(6), pose_eigen.segment(3, 3)));

        // Transform sampled points
        measurements_accessor_.at(fingertip_name) = finger_to_root * sampled_fingertips_.at(fingertip_name).colwise().homogeneous();

        // Accessors write directly inside the variable measurement_
    }

    return true;
}


void iCubHandContactsModel::printContactDetection()
{
    bool valid_detection;
    std::unordered_map<std::string, bool> fingers_contacts;
    std::tie(valid_detection, fingers_contacts) = contact_detection_->getActiveFingers();


    for (std::size_t i = 0; i < used_fingers_.size(); i++)
        if (fingers_contacts[used_fingers_.at(i)])
            std::cout << "In contact:" << used_fingers_.at(i) << std::endl;
}


Eigen::VectorXd iCubHandContactsModel::measure() const
{
    std::unordered_map<std::string, bool> active_fingers;
    std::tie(std::ignore, active_fingers) = contact_detection_->getActiveFingers();

    // Temporary
    // Consider that there is contact if at least one finger is in contact, it is enough in a grapsing scenario
    bool is_contact = false;
    for (std::size_t i = 0; i < used_fingers_.size(); i++)
        is_contact |= active_fingers.at(used_fingers_.at(i));

    VectorXd no_measurements = VectorXd(0);
    const VectorXd& effective_measurements = is_contact ? measurements_ : no_measurements;

    return effective_measurements;
}


std::string iCubHandContactsModel::getFingerTipName(const std::string finger_name)
{
    std::string fingertip_name = finger_name;

    if (finger_name == "middle")
    {
        fingertip_name += "3";
    }
    else
    {
        fingertip_name += "4";
    }

    return fingertip_name;
}


bool iCubHandContactsModel::isHandPartEnabled(const std::string hand_part_name)
{
    bool found = false;

    for (auto used_finger : used_fingers_)
    {
        if (hand_part_name.find(used_finger) != string::npos)
        {
            found = true;
            break;
        }
    }

    return found;
}


bool iCubHandContactsModel::loadMesh(const std::string hand_part_name, const std::string mesh_path)
{
    MeshImporter importer(mesh_path);

    // Convert mesh using MeshImporter
    std::istringstream mesh_input;
    bool valid_mesh;

    std::tie(valid_mesh, mesh_input) = importer.getMesh("obj");

    if (!valid_mesh)
    {
        yError() << log_ID_<< "ICUBHANDCONTACTSMODEL::CTOR::ERROR\n\tError: cannot load mesh file for hand part " + hand_part_name + ".";

        return false;
    }

    // Allocate storage for the mesh
    simpleTriMesh& mesh = hand_meshes_[hand_part_name];

    // Open converted obj using vcg mesh importer
    OBJImportInfo info;
    int outcome;
    outcome = simpleTriMeshImporter::OpenStream(mesh, mesh_input, info);

    if(simpleTriMeshImporter::ErrorCritical(outcome))
    {
        yError() << log_ID_<<  "ICUBHANDCONTACTSMODEL::CTOR::ERROR\n\tError: cannot load mesh file " + mesh_path +
                               ". Error:" + std::string(simpleTriMeshImporter::ErrorMsg(outcome)) + ".";
        return false;
    }

    // Update bounding box
    vcg::tri::UpdateBounding<simpleTriMesh>::Box(mesh);

    // Update face normals
    if(mesh.fn > 0)
        vcg::tri::UpdateNormal<simpleTriMesh>::PerFace(mesh);

    // Update vertex normals
    if(mesh.vn > 0)
	vcg::tri::UpdateNormal<simpleTriMesh>::PerVertex(mesh);

    return true;
}


void iCubHandContactsModel::sampleFingerTip(const std::string fingertip_name)
{
    simpleTriMesh& mesh = hand_meshes_.at(fingertip_name + "cleaned");

    // Perform back-face culling in order to remove faces pointing towards
    // the negative local z-axis (i.e. negative approach direction of the fingertip).
    // This way it is possible to sample points only on the surface of the fingertip
    // that may experience contacts with the external environment.
    Vector3d normal(3);
    Vector3d vertex0(3);
    Vector3d observer(3);
    observer << 0.0, 0.0, 0.1;
    for (FaceIterator fi = mesh.face.begin(); fi != mesh.face.end(); fi++)
    {
        // Extract face normal
        const auto n = fi->cN();
        normal << n[0], n[1], n[2];

        // Extract position of first vertex of the face
        const auto v = fi->cP(0);
        vertex0 << v[0], v[1], v[2];

        // Eval vector from view point to first vertex
        VectorXd diff = vertex0 - observer;

        // Perform culling check
        if (diff.dot(normal) >= 0)
            // Remove face
            simpleTriMeshAllocator::DeleteFace(mesh, *fi);
    }
    // Perform face garbage collection after culling
    simpleTriMeshAllocator::CompactFaceVector(mesh);

    // Perform Disk Poisson Sampling

    // Some default parametrs as found in MeshLab
    std::size_t oversampling = 20;
    triMeshSurfSampler::PoissonDiskParam poiss_params;
    poiss_params.radiusVariance = 1;
    poiss_params.geodesicDistanceFlag = false;
    poiss_params.bestSampleChoiceFlag = true;
    poiss_params.bestSamplePoolSize = 10;

    // Estimate radius required to obtain disk poisson sampling
    // with the number_of_points points
    simpleTriMesh::ScalarType radius;
    radius = triMeshSurfSampler::ComputePoissonDiskRadius(mesh, fingertip_number_samples_);

    // Generate preliminar montecarlo sampling with uniform probability
    simpleTriMesh montecarlo_mesh;
    triMeshSampler mc_sampler(montecarlo_mesh);
    mc_sampler.qualitySampling=true;
    triMeshSurfSampler::Montecarlo(mesh,
				   mc_sampler,
				   fingertip_number_samples_ * oversampling);
    // Copy the bounding box from the original mesh
    montecarlo_mesh.bbox = mesh.bbox;

    // Generate disk poisson samples by pruning the montecarlo cloud
    simpleTriMesh poiss_mesh;
    triMeshSampler dp_sampler(poiss_mesh);
    triMeshSurfSampler::PoissonDiskPruning(dp_sampler,
					   montecarlo_mesh,
					   radius,
					   poiss_params);
    vcg::tri::UpdateBounding<simpleTriMesh>::Box(poiss_mesh);

    // Store the cloud
    std::size_t number_points = std::distance(poiss_mesh.vert.begin(), poiss_mesh.vert.end());
    MatrixXd cloud(3, number_points);
    std::size_t i = 0;
    for (VertexIterator vi = poiss_mesh.vert.begin(); vi != poiss_mesh.vert.end(); vi++)
    {
	// Extract the point
	const auto p = vi->cP();

        // Add to the cloud
        cloud.col(i) << p[0], p[1], p[2];

        i++;
    }

    sampled_fingertips_[fingertip_name] = cloud;
}

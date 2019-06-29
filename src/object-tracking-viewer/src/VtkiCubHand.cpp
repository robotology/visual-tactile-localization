/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <VtkiCubHand.h>

#include <SuperimposeMesh/SICAD.h>

#include <yarp/eigen/Eigen.h>

using namespace Eigen;
using namespace yarp::eigen;


VtkiCubHand::VtkiCubHand(const std::string port_prefix, const std::string laterality) :
    hand_model_(true, false, laterality, "object-tracking", port_prefix)
{
    if (laterality != "right")
    {
        std::string err = log_ID_ + "::CTOR::ERROR\n\tError: the left hand is not implented.";
        throw(std::runtime_error(err));
    }

    // Try to open the hand pose input port
    if (!(hand_pose_port_in.open("/" + port_prefix + "/hand_pose:i")))
    {
        std::string err = log_ID_ + "::CTOR::ERROR\n\tError: cannot open hand pose input port.";
        throw(std::runtime_error(err));
    }

    // Retrieve icub hand mesh paths
    bool valid_paths;
    SICAD::ModelPathContainer meshes_paths;
    std::tie(valid_paths, meshes_paths) = hand_model_.getMeshPaths();

    if (!valid_paths)
    {
        std::string err = log_ID_ + "::CTOR::ERROR\n\tError: cannot load iCub hand meshes paths.";
        throw(std::runtime_error(err));
    }

    // Load all the meshes
    for (auto path : meshes_paths)
        meshes_.emplace(std::make_pair(path.first, VtkMesh(path.second)));
}


VtkiCubHand::~VtkiCubHand()
{
    // Close ports
    hand_pose_port_in.close();
}


void VtkiCubHand::addToRenderer(vtkRenderer& renderer)
{
    for (auto mesh : meshes_)
        meshes_.at(mesh.first).addToRenderer(renderer);
}


bool VtkiCubHand::updateHandPose(const Transform<double, 3, Affine>& offset)
{
    // Get the hand of the pose
    yarp::sig::Vector* hand_pose_yarp = hand_pose_port_in.read(false);

    if (hand_pose_yarp == nullptr)
        return false;

    VectorXd hand_pose = toEigen(*hand_pose_yarp);

    // Apply offset to the pose
    AngleAxisd orientation(hand_pose(6), hand_pose.segment<3>(3));
    Transform<double, 3, Affine> transform;
    transform = Translation<double, 3>(hand_pose.head<3>());
    transform.rotate(orientation);
    Transform<double, 3, Affine> transform_w_offset = offset * transform;

    // Convert to x-y-z-axis-angle
    VectorXd hand_pose_w_offset(7);
    hand_pose_w_offset.head<3>() = transform_w_offset.translation();
    AngleAxisd orientation_w_offset(transform_w_offset.rotation());
    hand_pose_w_offset.segment<3>(3) = orientation_w_offset.axis();
    hand_pose_w_offset(6) = orientation_w_offset.angle();

    // Get the pose of all parts of the hand according to finger forward kinematics
    bool valid_hand_parts_poses;
    std::vector<Superimpose::ModelPoseContainer> vector_poses;
    std::tie(valid_hand_parts_poses, vector_poses) = hand_model_.getModelPose(hand_pose_w_offset);

    if (!valid_hand_parts_poses)
        return false;

    // Update the pose of each part of the hand
    Superimpose::ModelPoseContainer& poses = vector_poses[0];
    for (auto& pose : poses)
    {
        const Superimpose::ModelPose& pose_si = poses.find(pose.first)->second;
        Map<const VectorXd> pose_eigen(pose_si.data(), 7);

        meshes_.at(pose.first).setPose(pose_eigen);
    }

    return true;
}

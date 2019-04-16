/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <CameraParameters.h>
#include <ObjectRenderer.h>


using namespace Eigen;

ObjectRenderer::ObjectRenderer
(
    const std::string& object_mesh_path,
    const std::string& sicad_shader_path,
    Camera& camera
)
{
    // Get camera parameters
    bool valid_camera_parameters;
    CameraParameters camera_parameters;
    std::tie(valid_camera_parameters, camera_parameters) = camera.getIntrinsicParameters();
    if (!valid_camera_parameters)
    {
        std::string err = log_ID_ + "::ctor. Cannot get camera intrinsic parameters.";
        throw(std::runtime_error(err));
    }

    // Configure superimposition engine
    SICAD::ModelPathContainer path_container;
    path_container.emplace("object", object_mesh_path);

    object_sicad_ = std::unique_ptr<SICAD>
    (
        new SICAD(path_container,
                  camera_parameters.width,
                  camera_parameters.height,
                  camera_parameters.fx,
                  camera_parameters.fy,
                  camera_parameters.cx,
                  camera_parameters.cy,
                  1,
                  sicad_shader_path,
                  {1.0, 0.0, 0.0, static_cast<float>(M_PI)})
    );
}


ObjectRenderer::~ObjectRenderer()
{ }


std::pair<bool, cv::Mat> ObjectRenderer::renderObject(const Transform<double, 3, Affine>& object_pose, const Eigen::Transform<double, 3, Eigen::Affine>& camera_pose)
{
    Superimpose::ModelPose si_pose;
    si_pose.resize(7);

    // Set object position
    const Vector3d& position = object_pose.translation();
    si_pose[0] = position[0];
    si_pose[1] = position[1];
    si_pose[2] = position[2];

    // Set object rotation
    AngleAxisd angle_axis = AngleAxisd(object_pose.rotation());
    si_pose[3] = angle_axis.axis()(0);
    si_pose[4] = angle_axis.axis()(1);
    si_pose[5] = angle_axis.axis()(2);
    si_pose[6] = angle_axis.angle();

    Superimpose::ModelPoseContainer si_pose_container;
    si_pose_container.emplace("object", si_pose);

    const Vector3d& camera_position = camera_pose.translation();
    AngleAxisd camera_angle_axis = AngleAxisd(camera_pose.rotation());
    VectorXd camera_axis_angle(4);
    camera_axis_angle.head<3>() = camera_angle_axis.axis();
    camera_axis_angle(3) = camera_angle_axis.angle();

    // Project mesh as a white blob onto the camera plane
    bool valid_superimpose = false;
    cv::Mat object_render;
    valid_superimpose = object_sicad_->superimpose(si_pose_container, camera_position.data(), camera_axis_angle.data(), object_render);

    if (!valid_superimpose)
        return std::make_pair(false, cv::Mat());

    return std::make_pair(true, object_render);
}

#include <iCubHandOcclusion.h>

#include <yarp/eigen/Eigen.h>

using namespace Eigen;
using namespace yarp::eigen;


iCubHandOcclusion::iCubHandOcclusion
(
    std::unique_ptr<iCubArmModel> icub_arm_model,
    const std::string port_prefix,
    const std::string eye_name
) :
    ObjectOcclusion(std::move(icub_arm_model), "convex_hull"),
    gaze_(port_prefix),
    eye_name_(eye_name)
{
    // Try to open the hand pose input port
    if (!(hand_pose_port_in.open("/" + port_prefix + "/hand_pose:i")))
    {
        std::string err = "ICUBHANDOCCLUSION::CTOR::ERROR\n\tError: cannot open hand pose input port.";
        throw(std::runtime_error(err));
    }

    // Retrieve the camera
    GazeController gaze();
    double icub_cam_fx;
    double icub_cam_fy;
    double icub_cam_cx;
    double icub_cam_cy;
    if(!gaze_.getCameraIntrinsics(eye_name, icub_cam_fx, icub_cam_fy, icub_cam_cx, icub_cam_cy))
    {
        std::string err = "ICUBHANDOCCLUSION::CTOR::ERROR\n\tError: cannot open retrieve icub camera instrinc parameters.";
        throw(std::runtime_error(err));
    }

    // Initialize sicad engine
    bool valid_mesh_path;
    SICAD::ModelPathContainer mesh_path;
    std::tie(valid_mesh_path, mesh_path) = mesh_model_->getMeshPaths();
    if (!valid_mesh_path)
    {
        std::string err = "ICUBHANDOCCLUSION::CTOR::ERROR\n\tError: cannot retrieve paths containing the hand meshes.";
        throw(std::runtime_error(err));
    }

    bool valid_shader_path;
    std::string shader_path;
    std::tie(valid_shader_path, shader_path) = mesh_model_->getShaderPaths();
    if (!valid_shader_path)
    {
        std::string err = "ICUBHANDOCCLUSION::CTOR::ERROR\n\tError: cannot retrieve paths containing the sicad engine shaders.";
        throw(std::runtime_error(err));
    }

    object_sicad_ = std::unique_ptr<SICAD>
        (
            new SICAD(mesh_path,
                      icub_cam_width_,
                      icub_cam_height_,
                      icub_cam_fx,
                      icub_cam_fy,
                      icub_cam_cx,
                      icub_cam_cy,
                      1,
                      shader_path,
                      {1.0, 0.0, 0.0, static_cast<float>(M_PI)})
            );
}


iCubHandOcclusion::~iCubHandOcclusion()
{
    hand_pose_port_in.close();
}


std::pair<bool, MatrixXd> iCubHandOcclusion::getOcclusionPose()
{
    // TODO: maybe it is better to store the previous pose
    // and return this in case of missing read

    yarp::sig::Vector* hand_pose = hand_pose_port_in.read(false);

    if (hand_pose == nullptr)
        return std::make_pair(false, MatrixXd());

    MatrixXd pose = toEigen(*hand_pose);

    return std::make_pair(true, pose);
}

std::tuple<bool, VectorXd, VectorXd> iCubHandOcclusion::getCameraPose()
{
    yarp::sig::Vector eye_pos_left;
    yarp::sig::Vector eye_att_left;
    yarp::sig::Vector eye_pos_right;
    yarp::sig::Vector eye_att_right;

    if (!gaze_.getCameraPoses(eye_pos_left, eye_att_left, eye_pos_right, eye_att_right))
        return std::make_tuple(false, VectorXd(), VectorXd());

    VectorXd eye_pos;
    VectorXd eye_att;
    if (eye_name_ == "left")
    {
        eye_pos = toEigen(eye_pos_left);
        eye_att = toEigen(eye_att_left);
    }
    else if (eye_name_ == "right")
    {
        eye_pos = toEigen(eye_pos_right);
        eye_att = toEigen(eye_att_right);
    }

    return std::make_tuple(true, eye_pos, eye_att);
}

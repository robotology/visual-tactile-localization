/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <CameraParameters.h>
#include <Viewer.h>

#include <vtkTransform.h>

#include <string>

#include <yarp/eigen/Eigen.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>

#include <Eigen/Dense>

#include <chrono>

using namespace yarp::eigen;
using namespace yarp::os;
using namespace yarp::sig;
using namespace Eigen;

Viewer::Viewer(const std::string port_prefix, ResourceFinder& rf)
{
    // Open RPC input port for commands
    if (!port_rpc_command_.open("/object-tracking-viewer/cmd:i"))
    {
        std::string err = log_ID_ + "::ctor. Error: cannot open RPC input command port.";
        throw(std::runtime_error(err));
    }

    if (!(this->yarp().attachAsServer(port_rpc_command_)))
    {
        std::string err = "PFILTER::CTOR::ERROR\n\tError: cannot attach the RPC command port.";
        throw(std::runtime_error(err));
    }

    // Load period
    const int period = static_cast<int>(rf.check("period", Value(1.0)).asDouble() * 1000);
    yInfo() << log_ID_ << "- period:" << period << "ms";

    // Load point cloud z threshold
    pc_left_z_threshold_  = rf.check("pc_left_z_thr", Value(1.0)).asDouble();
    yInfo() << log_ID_ << "- pc_left_z_thr:" << pc_left_z_threshold_;

    // Load hand visualization boolean
    show_hand_ = rf.check("show_hand", Value(false)).asBool();
    yInfo() << log_ID_ << "- show_hand:" << show_hand_;

    hand_in_camera_frame_ = rf.check("hand_in_camera_frame", Value(false)).asBool();
    yInfo() << log_ID_ << "- hand_in_camera_frame:" << hand_in_camera_frame_;

    // Load hand laterality
    std::string hand_laterality = rf.check("hand_laterality", Value("right")).asString();
    yInfo() << log_ID_ << "- hand_laterality:" << hand_laterality;

    // Load ground truth visualization boolean
    show_ground_truth_ = rf.check("show_ground_truth", Value("false")).asBool();
    yInfo() << log_ID_ << "- show_ground_truth:" << show_ground_truth_;

    // Load estimate visualization boolean
    show_estimate_ = rf.check("show_estimate", Value("false")).asBool();
    yInfo() << log_ID_ << "- show_estimate:" << show_estimate_;

    // Load point cloud visualization boolean
    show_point_cloud_ = rf.check("show_point_cloud", Value("false")).asBool();
    yInfo() << log_ID_ << "- show_point_cloud:" << show_point_cloud_;

    // Load name of the camera
    std::string camera_name = rf.check("camera", Value("icub")).asString();
    yInfo() << log_ID_ << "- camera:" << camera_name;

    // Load key for fallback camera configuration
    std::string camera_fallback_key = rf.check("fallback_config", Value("left_320_240")).asString();
    yInfo() << log_ID_ << "- camera fallback configuration key:" << camera_fallback_key;

    // Load matched configuration file within context object-tracking
    std::string object_tracking_configuration_file = rf.check("object_tracking_config", Value("config.ini")).asString();

    if (show_ground_truth_)
    {
        // Open estimate input port
        if(!port_ground_truth_in_.open("/" + port_prefix + "/ground-truth:i"))
        {
            std::string err = "VIEWER::CTOR::ERROR\n\tError: cannot open ground truth input port.";
            throw(std::runtime_error(err));
        }
    }

    if (show_estimate_)
    {
        // Open estimate input port
        if(!port_estimate_in_.open("/" + port_prefix + "/estimate:i"))
        {
            std::string err = "VIEWER::CTOR::ERROR\n\tError: cannot open estimate input port.";
            throw(std::runtime_error(err));
        }
    }

    // Load mesh from the default configuration file of module object-tracking
    ResourceFinder rf_object_tracking;
    rf_object_tracking.setVerbose(true);
    rf_object_tracking.setDefaultContext("object-tracking");
    rf_object_tracking.setDefaultConfigFile(object_tracking_configuration_file.c_str());
    rf_object_tracking.configure(0, NULL);

    ResourceFinder rf_object = rf_object_tracking.findNestedResourceFinder("OBJECT");
    const std::string object_name = rf_object.check("object_name", Value("ycb_mustard")).asString();
    yInfo() << log_ID_ << "- object_name:" << object_name;

    const std::string mesh_path = rf_object_tracking.findPath("mesh/" + object_name) + "/nontextured.ply";
    yInfo() << log_ID_ << "- mesh_path:" << mesh_path;

    reader_ = vtkSmartPointer<vtkPLYReader>::New();
    reader_->SetFileName(mesh_path.c_str());
    reader_->Update();

    // Initialize camera
    if (camera_name == "iCubCamera")
        camera_ = std::unique_ptr<iCubCamera>(new iCubCamera("left", port_prefix, "object-tracking", camera_fallback_key));
    else if (camera_name == "RealsenseCamera")
        camera_ = std::unique_ptr<RealsenseCamera>(new RealsenseCamera(port_prefix, "object-tracking", camera_fallback_key));
    else
    {
        std::string err = "VIEWER::CTOR::ERROR\n\tError: the camera you requested (" + camera_name + ") is not supported.";
        throw(std::runtime_error(err));
    }
    camera_->initialize();

    // Cache deprojection matrix
    bool valid_deprojection_matrix;
    std::tie(valid_deprojection_matrix, deprojection_matrix_) = camera_->getDeprojectionMatrix();
    if(!valid_deprojection_matrix)
    {
        std::string err = "VIEWER::CTOR::ERROR\n\tError: cannot get deprojection matrix from the camera.";
        throw(std::runtime_error(err));
    }

    // Cache 2d coordinates
    std::size_t u_stride = 1;
    std::size_t v_stride = 1;
    set2DCoordinates(u_stride, v_stride);

    // Configure mesh actor
    mapper_ = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper_->SetInputConnection(reader_->GetOutputPort());

    if (show_estimate_)
    {
        mesh_actor_ = vtkSmartPointer<vtkActor>::New();
        mesh_actor_->SetMapper(mapper_);
    }

    if (show_ground_truth_)
    {
        mesh_actor_ground_truth_ = vtkSmartPointer<vtkActor>::New();
        mesh_actor_ground_truth_->SetMapper(mapper_);
        mesh_actor_ground_truth_->GetProperty()->SetColor(0.0, 0.8, 0.0);
        mesh_actor_ground_truth_->GetProperty()->SetOpacity(0.4);
    }

    // Configure measurements actor
    if (show_point_cloud_)
    {
        int points_size = 2;
        vtk_measurements_ = std::unique_ptr<Points>(new Points(points_size));
    }

    // Configure vtk hand actors
    if (show_hand_)
        vtk_icub_hand_ = std::unique_ptr<VtkiCubHand>(new VtkiCubHand(port_prefix + "/hand/" + hand_laterality, hand_laterality));

    // Configure axes
    axes_ = vtkSmartPointer<vtkAxesActor>::New();
    orientation_widget_ = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
    orientation_widget_->SetOrientationMarker(axes_);

    // Configure camera
    vtk_camera_ = vtkSmartPointer<vtkCamera>::New();
    vtk_camera_->SetPosition(0.0, 0.0, 0.5);
    vtk_camera_->SetViewUp(-1.0, 0.0, -1.0);

    // Configure interactor style
    interactor_style_ = vtkSmartPointer<vtkInteractorStyleSwitch>::New();
    interactor_style_->SetCurrentStyleToTrackballCamera();

    // Rendering
    renderer_ = vtkSmartPointer<vtkRenderer>::New();
    render_window_ = vtkSmartPointer<vtkRenderWindow>::New();
    render_window_interactor_ = vtkSmartPointer<vtkRenderWindowInteractor>::New();

    render_window_->AddRenderer(renderer_);
    render_window_->SetSize(600,600);
    render_window_interactor_->SetRenderWindow(render_window_);

    // Add actors to renderer
    if (show_estimate_)
        renderer_->AddActor(mesh_actor_);
    if (show_point_cloud_)
        renderer_->AddActor(vtk_measurements_->get_actor());
    if (show_hand_)
        vtk_icub_hand_->addToRenderer(*renderer_);
    if (show_ground_truth_)
        renderer_->AddActor(mesh_actor_ground_truth_);

    renderer_->SetBackground(0.8, 0.8, 0.8);

    // Activate camera
    renderer_->SetActiveCamera(vtk_camera_);

    // Set interactor style
    render_window_interactor_->SetInteractorStyle(interactor_style_);

    // Enable orientation widget
    orientation_widget_->SetInteractor(render_window_interactor_);
    orientation_widget_->SetEnabled(1);
    orientation_widget_->InteractiveOn();

    render_window_->Render();

    // Initialize must be called prior to creating timer events.
    render_window_interactor_->Initialize();
    render_window_interactor_->CreateRepeatingTimer(period);

    vtkSmartPointer<vtkUpdate> update_callback = vtkSmartPointer<vtkUpdate>::New();
    update_callback->setViewer(this);
    render_window_interactor_->AddObserver(vtkCommand::TimerEvent, update_callback);

    // Start the interaction and timer
    render_window_interactor_->Start();
}


Viewer::~Viewer()
{
    port_estimate_in_.close();
    port_rpc_command_.close();
}


void Viewer::updateView()
{
    mutex_rpc_.lock();
    if (set_new_superquadric_)
    {
        enableSuperquadricActor();

        set_new_superquadric_ = false;
    }
    mutex_rpc_.unlock();

    // Update estimate
    if (show_estimate_)
    {
        yarp::sig::Vector* estimate = port_estimate_in_.read(false);

        if (estimate != nullptr)
        {

            VectorXd state = toEigen(*estimate);

            // Create a new transform
            vtkSmartPointer<vtkTransform> vtk_transform = vtkSmartPointer<vtkTransform>::New();

            // Set translation
            vtk_transform->Translate(state.head<3>().data());

            // Set rotation
            vtk_transform->RotateWXYZ(state(6) * 180 / M_PI,
                                      state(3), state(4), state(5));

            if (use_superquadric_visualization_)
                vtk_transform->RotateX(-90.0);

            // Apply transform

            mesh_actor_->SetUserTransform(vtk_transform);
        }
    }

    // Update ground truth
    if (show_ground_truth_)
    {
        yarp::sig::Vector* ground_truth = port_ground_truth_in_.read(false);

        if (ground_truth != nullptr)
        {
            VectorXd state = toEigen(*ground_truth);

            // Create a new transform
            vtkSmartPointer<vtkTransform> vtk_transform = vtkSmartPointer<vtkTransform>::New();

            // Set translation
            vtk_transform->Translate(state.head<3>().data());

            // Set rotation
            vtk_transform->RotateWXYZ(state(6) * 180 / M_PI,
                                      state(3), state(4), state(5));
            // Apply transform
            mesh_actor_ground_truth_->SetUserTransform(vtk_transform);
        }
    }

    // Update point cloud of the scene
    if (show_point_cloud_)
    {
        bool valid_rgb;
        cv::Mat rgb_image;
        std::tie(valid_rgb, rgb_image) = camera_->getRgbImage(false);

        bool valid_depth;
        MatrixXf depth_image;
        std::tie(valid_depth, depth_image) = camera_->getDepthImage(false);

        if (valid_rgb && valid_depth)
        {
            // Try to get the point cloud
            bool valid_point_cloud;
            Eigen::MatrixXd point_cloud;
            Eigen::VectorXi valid_coordinates;

            std::tie(valid_point_cloud, point_cloud, valid_coordinates) = get3DPointCloud(depth_image, pc_left_z_threshold_);

            if (valid_point_cloud)
            {
                // Set 3D points
                vtk_measurements_->set_points(point_cloud);
                // Extract corresponding colors from the rgb image
                vtk_measurements_->set_colors(coordinates_2d_, valid_coordinates, rgb_image);
            }
        }
    }

    // Update hand of the robot
    if (show_hand_)
    {
        if (hand_in_camera_frame_)
        {
            bool valid_pose;
            Transform<double, 3, Affine> camera_pose;

            std::tie(valid_pose, camera_pose) = camera_->getCameraPose(false);
            if (valid_pose)
                vtk_icub_hand_->updateHandPose(camera_pose);
        }
        else
            vtk_icub_hand_->updateHandPose();
    }
}


void Viewer::set2DCoordinates(const std::size_t u_stride, const std::size_t v_stride)
{
    // Get camera parameters
    CameraParameters parameters;
    std::tie(std::ignore, parameters) = camera_->getIntrinsicParameters();

    for (std::size_t u = 0; u < parameters.width; u += u_stride)
    {
        for (std::size_t v = 0; v < parameters.height; v += v_stride)
        {
            coordinates_2d_.push_back(std::make_pair(u, v));
        }
    }
}


std::tuple<bool, Eigen::MatrixXd, Eigen::VectorXi> Viewer::get3DPointCloud(const MatrixXf& depth, const float z_threshold)
{
    // Get the camera pose
    bool valid_camera_pose;
    Eigen::Transform<double, 3, Eigen::Affine> camera_pose;
    std::tie(valid_camera_pose, camera_pose) = camera_->getCameraPose(true);
    if (!valid_camera_pose)
        return std::make_tuple(false, MatrixXd(), VectorXi());

    // Valid points mask
    Eigen::VectorXi valid_points(coordinates_2d_.size());

    for (std::size_t i = 0; i < valid_points.size(); i++)
    {
        valid_points(i) = 0;

        float depth_u_v = depth(coordinates_2d_[i].second, coordinates_2d_[i].first);
        if ((depth_u_v > 0) && (depth_u_v < z_threshold))
            valid_points(i) = 1;
    }

    std::size_t num_valids = valid_points.sum();
    if (num_valids == 0)
        return std::make_tuple(false, MatrixXd(), VectorXi());

    // Compose 3d points with respect to left camera referece frame
    Eigen::MatrixXd points(3, num_valids);
    for (int i = 0, j = 0; i < coordinates_2d_.size(); i++)
    {
        if(valid_points(i) == 1)
        {
            float depth_u_v = depth(coordinates_2d_[i].second, coordinates_2d_[i].first);
            points.col(j) = deprojection_matrix_.col(i) * depth_u_v;

            j++;
        }
    }

    // Points with respect to robot root frame
    MatrixXd points_robot = camera_pose * points.colwise().homogeneous();

    return std::make_tuple(true, points_robot, valid_points);
}


void Viewer::enableSuperquadricActor()
{
    use_superquadric_visualization_ = true;

    // Remove the previously added actor
    renderer_->RemoveActor(mesh_actor_);

    double x = superquadric_parameters_(0);
    double y = superquadric_parameters_(1);
    double z = superquadric_parameters_(2);
    double phi = superquadric_parameters_(3);
    double theta = superquadric_parameters_(4);
    double psi = superquadric_parameters_(5);
    double size_x = superquadric_parameters_(6);
    double size_y = superquadric_parameters_(7);
    double size_z = superquadric_parameters_(8);
    double eps_1 = superquadric_parameters_(9);
    double eps_2 = superquadric_parameters_(10);

    // Create a new supequadric actor
    superquadric_ = vtkSmartPointer<vtkSuperquadric>::New();
    superquadric_->ToroidalOff();
    superquadric_->SetSize(1.0);
    superquadric_->SetCenter(0.0, 0.0, 0.0);
    superquadric_->SetScale(size_x, size_y, size_z);
    superquadric_->SetPhiRoundness(eps_1);
    superquadric_->SetThetaRoundness(eps_2);

    superquadric_sample_ = vtkSmartPointer<vtkSampleFunction>::New();
    superquadric_sample_->SetSampleDimensions(50,50,50);
    superquadric_sample_->SetImplicitFunction(superquadric_);
    superquadric_sample_->SetModelBounds(-size_x, size_x, -size_y, size_y, -size_z, size_z);

    superquadric_contours_ = vtkSmartPointer<vtkContourFilter>::New();
    superquadric_contours_->SetInputConnection(superquadric_sample_->GetOutputPort());
    superquadric_contours_->GenerateValues(1,0.0,0.0);

    superquadric_mapper_ = vtkSmartPointer<vtkPolyDataMapper>::New();
    superquadric_mapper_->SetInputConnection(superquadric_contours_->GetOutputPort());
    superquadric_mapper_->ScalarVisibilityOff();

    mesh_actor_ = vtkSmartPointer<vtkActor>::New();
    mesh_actor_->SetMapper(superquadric_mapper_);

    vtkSmartPointer<vtkTransform> vtk_transform = vtkSmartPointer<vtkTransform>::New();

    // Set translation
    Vector3d position;
    position << x, y, z;
    vtk_transform->Translate(position.data());

    // Set rotation
    AngleAxisd angle_axis(AngleAxisd(phi, Vector3d::UnitZ()) *
                          AngleAxisd(theta, Vector3d::UnitY()) *
                          AngleAxisd(psi, Vector3d::UnitX()));
    Vector3d axis = angle_axis.axis();
    vtk_transform->RotateWXYZ(angle_axis.angle() * 180 / M_PI,
                              axis(0), axis(1), axis(2));
    vtk_transform->RotateX(-90.0);

    // Set pose
    mesh_actor_->SetUserTransform(vtk_transform);

    // Add the new actor
    renderer_->AddActor(mesh_actor_);
}



bool Viewer::use_superquadric
(
    const double size_x,
    const double size_y,
    const double size_z,
    const double eps_1,
    const double eps_2,
    const double x,
    const double y,
    const double z,
    const double phi,
    const double theta,
    const double psi
)
{
    mutex_rpc_.lock();

    superquadric_parameters_.resize(11);
    superquadric_parameters_(0) = x;
    superquadric_parameters_(1) = y;
    superquadric_parameters_(2) = z;
    superquadric_parameters_(3) = phi;
    superquadric_parameters_(4) = theta;
    superquadric_parameters_(5) = psi;
    superquadric_parameters_(6) = size_x;
    superquadric_parameters_(7) = size_y;
    superquadric_parameters_(8) = size_z;
    superquadric_parameters_(9) = eps_1;
    superquadric_parameters_(10) = eps_2;

    set_new_superquadric_ = true;

    mutex_rpc_.unlock();

    return true;
}

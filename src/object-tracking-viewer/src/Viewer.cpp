#include <Viewer.h>

#include <vtkTransform.h>

#include <string>

#include <yarp/eigen/Eigen.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>

#include <Eigen/Dense>

using namespace yarp::eigen;
using namespace yarp::os;
using namespace yarp::sig;
using namespace Eigen;

Viewer::Viewer(const std::string port_prefix, ResourceFinder& rf) :
    sfm_(port_prefix)
{
    // Open estimate input port
    if(!port_estimate_in_.open("/" + port_prefix + "/estimate:i"))
    {
        std::string err = "VIEWER::CTOR::ERROR\n\tError: cannot open estimate input port.";
        throw(std::runtime_error(err));
    }

    // Open camera input port
    // (required to get coloured 3D point cloud)
    if(!port_image_in_.open("/" + port_prefix + "/cam/left:i"))
    {
        std::string err = "VIEWER::CTOR::ERROR\n\tError: cannot open left camera input port.";
        throw(std::runtime_error(err));
    }

    // Load period
    const int period = static_cast<int>(rf.check("period", Value(1.0)).asDouble() * 1000);

    // Load mesh
    const std::string object_name = rf.check("object_name", Value("ycb_mustard")).asString();

    ResourceFinder rf_object_tracking;
    rf_object_tracking.setVerbose(true);
    rf_object_tracking.setDefaultContext("object-tracking");
    const std::string mesh_path = rf_object_tracking.findPath("mesh/" + object_name) + "/nontextured.ply";

    reader_ = vtkSmartPointer<vtkPLYReader>::New();
    reader_->SetFileName(mesh_path.c_str());
    reader_->Update();

    // Configure SFM library.
    ResourceFinder rf_sfm;
    rf_sfm.setVerbose(true);
    rf_sfm.setDefaultConfigFile("sfm_config.ini");
    rf_sfm.setDefaultContext("object-tracking");
    rf_sfm.configure(0, NULL);

    if (!sfm_.configure(rf_sfm))
    {
        std::string err = "VIEWER::CTOR::ERROR\n\tError: cannot configure instance of SFM library.";
        throw(std::runtime_error(err));
    }

    // Set 2d coordinates
    std::size_t u_stride = 1;
    std::size_t v_stride = 1;
    set2DCoordinates(u_stride, v_stride);

    // Configure mesh actor
    mapper_ = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper_->SetInputConnection(reader_->GetOutputPort());

    mesh_actor_ = vtkSmartPointer<vtkActor>::New();
    mesh_actor_->SetMapper(mapper_);

    // Configure measurements actor
    int points_size = 2;
    vtk_measurements_ = unique_ptr<Points>(new Points(points_size));

    // Configure axes
    axes_ = vtkSmartPointer<vtkAxesActor>::New();
    orientation_widget_ = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
    orientation_widget_->SetOrientationMarker(axes_);

    // Configure camera
    camera_ = vtkSmartPointer<vtkCamera>::New();
    camera_->SetPosition(0.0, 0.0, 0.5);
    camera_->SetViewUp(-1.0, 0.0, -1.0);

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
    renderer_->AddActor(mesh_actor_);
    renderer_->AddActor(vtk_measurements_->get_actor());
    renderer_->SetBackground(0.8, 0.8, 0.8);

    // Activate camera
    renderer_->SetActiveCamera(camera_);

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
    port_image_in_.close();
}


void Viewer::updateView()
{
    // Update estimate
    {
        Vector* estimate = port_estimate_in_.read(false);

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
            // Apply transform
            mesh_actor_->SetUserTransform(vtk_transform);
        }
    }

    // Update point cloud of the scene
    {
        // Try to get input image
        yarp::sig::ImageOf<PixelRgb>* image_in = port_image_in_.read(false);

        if (image_in == nullptr)
            return;

        // Try to get the point cloud
        bool valid_point_cloud;
        Eigen::MatrixXd point_cloud;
        Eigen::VectorXi valid_coordinates;
        std::tie(valid_point_cloud, point_cloud, valid_coordinates) = sfm_.get3DPoints(coordinates_2d_, false, 1.0);

        if (!valid_point_cloud)
            return;

        // Set 3D points
        vtk_measurements_->set_points(point_cloud);
        // Extract corresponding colors from the rgb image
        vtk_measurements_->set_colors(coordinates_2d_, valid_coordinates, *image_in);
    }
}


void Viewer::set2DCoordinates(const std::size_t u_stride, const std::size_t v_stride)
{
    for (std::size_t u = 0; u < cam_width_; u += u_stride)
    {
        for (std::size_t v = 0; v < cam_height_; v += v_stride)
        {
            coordinates_2d_.push_back(std::make_pair(u, v));
        }
    }
}

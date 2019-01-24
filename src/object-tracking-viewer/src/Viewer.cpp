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

Viewer::Viewer(const std::string port_prefix, ResourceFinder& rf) //:
    //sfm_(port_prefix + "/SFM")
{
    // Open input port
    if(!port_estimate_in_.open("/" + port_prefix + "/estimate:i"))
    {
        std::string err = "VIEWER::CTOR::ERROR\n\tError: cannot open estimate input port.";
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

    // Configure mesh actor
    mapper_ = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper_->SetInputConnection(reader_->GetOutputPort());

    mesh_actor_ = vtkSmartPointer<vtkActor>::New();
    mesh_actor_->SetMapper(mapper_);

    // Configure measurements actor
    // renderer_->AddActor(vtk_measurements->get_actor());

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
}


void Viewer::updateView()
{
    Vector* estimate = port_estimate_in_.read(false);

    if (estimate == nullptr)
        return;
    
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
    
    // Set measurements
    // vtk_measurements->set_points(measurements_[step_]);
    // vtk_measurements->set_color("red");
}

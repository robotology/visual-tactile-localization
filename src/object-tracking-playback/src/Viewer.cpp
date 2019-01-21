#include <Viewer.h>

#include <Eigen/Geometry>

#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkTransform.h>

#include <fstream>
#include <memory>
#include <sstream>
#include <string>

using namespace Eigen;


class LeftRightArrowManualPlayback : public vtkInteractorStyleTrackballCamera
{
public:
    static LeftRightArrowManualPlayback* New();
    vtkTypeMacro(LeftRightArrowManualPlayback, vtkInteractorStyleTrackballCamera);


    void SetVisualizer(Visualizer* visualizer)
    {
        visualizer_ = visualizer;
    }


    virtual void OnKeyPress()
    {
        // Get the keypress.
        vtkRenderWindowInteractor *rwi = this->Interactor;
        std::string key = rwi->GetKeySym();

        // Move to the previous time step.
        if(key == "Left")
        {
            visualizer_->stepBackward();
        }

        // Move to the next time step.
        if(key == "Right")
        {
            visualizer_->stepForward();
        }

        // Forward events
        vtkInteractorStyleTrackballCamera::OnKeyPress();
    }
private:
    Visualizer* visualizer_;
};
vtkStandardNewMacro(LeftRightArrowManualPlayback);


std::pair<bool, MatrixXd> Visualizer::readStateFromFile(const std::string& filename, const std::size_t num_fields)
{
    MatrixXd data;

    std::ifstream istrm(filename);

    if (!istrm.is_open())
    {
        std::cout << "Failed to open " << filename << '\n';

        return std::make_pair(false, MatrixXd(0,0));
    }
    else
    {
        std::vector<std::string> istrm_strings;
        std::string line;
        while (std::getline(istrm, line))
        {
            istrm_strings.push_back(line);
        }

        data.resize(num_fields, istrm_strings.size());
        std::size_t found_lines = 0;
        for (auto line : istrm_strings)
        {
            std::size_t found_fields = 0;
            std::string number_str;
            std::istringstream iss(line);

            while (iss >> number_str)
            {
                std::size_t index = (num_fields * found_lines) + found_fields;
                *(data.data() + index) = std::stod(number_str);
                found_fields++;
            }
            if (num_fields != found_fields)
            {
                std::cout << "Malformed input file " << filename << '\n';

                return std::make_pair(false, MatrixXd(0,0));
            }
            found_lines++;
        }

        return std::make_pair(true, data);
    }
}


std::pair<bool, std::vector<MatrixXd>> Visualizer::readMeasurementsFromFile(const std::string& filename)
{
    std::vector<MatrixXd> data;

    std::ifstream istrm(filename);

    if (!istrm.is_open())
    {
        std::cout << "Failed to open " << filename << '\n';

        return std::make_pair(false, std::vector<MatrixXd>());
    }
    else
    {
        std::vector<std::string> istrm_strings;
        std::string line;
        while (std::getline(istrm, line))
        {
            istrm_strings.push_back(line);
        }

        istrm.close();

        for (auto line : istrm_strings)
        {
            std::istringstream iss(line);

            std::string number_str;
            std::vector<double> points;
            while (iss >> number_str)
                points.push_back(std::stod(number_str));

            if ((points.size() % 3) != 0)
            {
                std::cout << "Malformed input file " << filename << '\n';

                return std::make_pair(false, std::vector<MatrixXd>());
            }

            MatrixXd data_i = MatrixXd(3, points.size() / 3);
            for (std::size_t i = 0; i < points.size(); i++)
                *(data_i.data() + i) = points[i];

            data.push_back(data_i);
        }

        return std::make_pair(true, data);
    }
}


Visualizer::Visualizer
(
    const std::string mesh_filename,
    const std::string target_data_filename,
    const std::string estimate_data_filename,
    const std::string prediction_data_filename,
    const std::string measurements_data_filename
)
{
    // Load target data
    bool valid_target;
    std::tie(valid_target, target_) = readStateFromFile(target_data_filename, 13);
    if (!valid_target)
    {
        throw std::runtime_error("ERROR::VISUALIZER::CTOR\nERROR:Invalid target data.");
    }

    // Load estimate data
    bool valid_estimate;
    std::tie(valid_estimate, estimate_) = readStateFromFile(estimate_data_filename, 6);
    if (!valid_estimate)
    {
        throw std::runtime_error("ERROR::VISUALIZER::CTOR\nERROR:Invalid estimate data.");
    }

    // Load prediction data
    bool valid_prediction;
    std::tie(valid_prediction, prediction_) = readStateFromFile(prediction_data_filename, 6);
    if (!valid_prediction)
    {
        throw std::runtime_error("ERROR::VISUALIZER::CTOR\nERROR:Invalid prediction data.");
    }

    // Load measurements
    bool valid_measurements;
    std::tie(valid_measurements, measurements_) = readMeasurementsFromFile(measurements_data_filename);
    if (!valid_measurements)
    {
        throw std::runtime_error("ERROR::VISUALIZER::CTOR\nERROR:Invalid measurements data.");
    }
    vtk_measurements = std::unique_ptr<Points>(new Points(measurements_[0], 4));
    vtk_measurements->set_color("red");

    // Load mesh
    reader_ = vtkSmartPointer<vtkPLYReader>::New();
    reader_->SetFileName(mesh_filename.c_str());
    reader_->Update();

    // Visualize

    // Mesh part
    mapper_ = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper_->SetInputConnection(reader_->GetOutputPort());

    mesh_actor_ = vtkSmartPointer<vtkActor>::New();
    mesh_actor_->SetMapper(mapper_);

    estimate_actor_ = vtkSmartPointer<vtkActor>::New();
    estimate_actor_->SetMapper(mapper_);
    estimate_actor_->GetProperty()->SetColor(0.9, 0.0, 0.0);
    estimate_actor_->GetProperty()->SetOpacity(0.3);

    prediction_actor_ = vtkSmartPointer<vtkActor>::New();
    prediction_actor_->SetMapper(mapper_);
    prediction_actor_->GetProperty()->SetColor(0.0, 0.0, 0.9);
    prediction_actor_->GetProperty()->SetOpacity(0.3);

    renderer_ = vtkSmartPointer<vtkRenderer>::New();
    renderer_->AddActor(mesh_actor_);
    renderer_->AddActor(estimate_actor_);
    renderer_->AddActor(prediction_actor_);
    renderer_->SetBackground(0.8, 0.8, 0.8);

    // Measurements part
    renderer_->AddActor(vtk_measurements->get_actor());

    // Rendering
    renderWindow_ = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow_->AddRenderer(renderer_);

    renderWindowInteractor_ = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor_->SetRenderWindow(renderWindow_);

    // Set playback control from keyboard
    manualPlaybackCtrl_ = vtkSmartPointer<LeftRightArrowManualPlayback>::New();
    renderWindowInteractor_->SetInteractorStyle(manualPlaybackCtrl_);
    manualPlaybackCtrl_->SetCurrentRenderer(renderer_);
    manualPlaybackCtrl_->SetVisualizer(this);

    // Reset step index
    step_ = 0;

    // Process the first time step
    updateView();

    // Give the control to VTK
    renderWindowInteractor_->Start();
}


void Visualizer::stepForward()
{
    step_++;
    if (step_ >= target_.cols())
        step_ = (target_.cols() - 1);

    updateView();
}

void Visualizer::stepBackward()
{
    step_--;
    if (step_ < 0)
        step_ = 0;

    updateView();
}

void Visualizer::updateView()
{
    // Ground truth
    {
        // Get the current data
        Ref<VectorXd> state = target_.col(step_);
        Ref<Vector3d> pos = state.topRows(3);
        Ref<Vector4d> quaternion = state.segment(6, 4);

        // Create a new transform
        vtkSmartPointer<vtkTransform> vtk_transform = vtkSmartPointer<vtkTransform>::New();

        // Set translation
        vtk_transform->Translate(pos.data());

        // set rotation
        Quaterniond q;
        q.w() = quaternion(0);
        q.x() = quaternion(1);
        q.y() = quaternion(2);
        q.z() = quaternion(3);
        AngleAxisd angle_axis(q);
        Vector3d axis = angle_axis.axis();
        vtk_transform->RotateWXYZ(angle_axis.angle() * 180 / M_PI,
                                  axis(0), axis(1), axis(2));
        // Apply transform
        mesh_actor_->SetUserTransform(vtk_transform);
    }
    // Estimate
    {
        // Get the current data
        Ref<VectorXd> state = estimate_.col(step_);
        Ref<Vector3d> pos = state.topRows(3);
        Ref<Vector3d> euler = state.bottomRows(3);

        // Create a new transform
        vtkSmartPointer<vtkTransform> vtk_transform = vtkSmartPointer<vtkTransform>::New();

        // Set translation
        vtk_transform->Translate(pos.data());

        // Set rotation
        Quaterniond q = AngleAxis<double>(euler(0), Vector3d::UnitZ()) *
                        AngleAxis<double>(euler(1), Vector3d::UnitY()) *
                        AngleAxis<double>(euler(2), Vector3d::UnitX());
        AngleAxisd angle_axis(q);
        Vector3d axis = angle_axis.axis();
        vtk_transform->RotateWXYZ(angle_axis.angle() * 180 / M_PI,
                                  axis(0), axis(1), axis(2));
        // Apply transform
        estimate_actor_->SetUserTransform(vtk_transform);
    }
    // Prediction
    {
        // Get the current data
        Ref<VectorXd> state = prediction_.col(step_);
        Ref<Vector3d> pos = state.topRows(3);
        Ref<Vector3d> euler = state.bottomRows(3);

        // Create a new transform
        vtkSmartPointer<vtkTransform> vtk_transform = vtkSmartPointer<vtkTransform>::New();

        // Set translation
        vtk_transform->Translate(pos.data());

        // Set rotation
        Quaterniond q = AngleAxis<double>(euler(0), Vector3d::UnitZ()) *
                        AngleAxis<double>(euler(1), Vector3d::UnitY()) *
                        AngleAxis<double>(euler(2), Vector3d::UnitX());
        AngleAxisd angle_axis(q);
        Vector3d axis = angle_axis.axis();
        vtk_transform->RotateWXYZ(angle_axis.angle() * 180 / M_PI,
                                  axis(0), axis(1), axis(2));
        // Apply transform
        prediction_actor_->SetUserTransform(vtk_transform);
    }

    // Set measurements
    vtk_measurements->set_points(measurements_[step_]);
    vtk_measurements->set_color("red");

    // Render
    renderWindowInteractor_->Render();
}

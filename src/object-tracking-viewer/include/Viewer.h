#ifndef VIEWER_H
#define VIEWER_H

#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkCamera.h>
#include <vtkDoubleArray.h>
#include <vtkInteractorStyleSwitch.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkPLYReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkPointData.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkVertexGlyphFilter.h>

#include <Eigen/Dense>

#include <GazeController.h>

#include <VtkiCubHand.h>

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>

#include <memory>
#include <string>


class Object
{
protected:
    vtkSmartPointer<vtkPolyDataMapper> vtk_mapper;
    vtkSmartPointer<vtkActor> vtk_actor;

public:
    vtkSmartPointer<vtkActor> &get_actor()
    {
        return vtk_actor;
    }
};

// class vtkPointsEigen : public vtkPoints
// {
// public:
//     vtkPointsEigen() :
//         data_double_(vtkSmartPointer<vtkDoubleArray>::New())
//     { }

//     void set_data(Eigen::Ref<Eigen::MatrixXd> points)
//     {
//         Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>> points_row_major(points.data(), points.rows(), points.cols());

//         data_double_->SetArray(points_row_major.data(), points_row_major.size(), 1);
//         Data = data_double_.Get();
//     }

// private:
//     vtkSmartPointer<vtkDoubleArray> data_double_;
// };

class Points : public Object
{
protected:
    // std::shared_ptr<vtkPoints> vtk_points;
    // std::shared_ptr<vtkPointsEigen> vtk_points_eigen;
    vtkSmartPointer<vtkPoints> vtk_points;
    vtkSmartPointer<vtkUnsignedCharArray> vtk_colors;
    vtkSmartPointer<vtkPolyData> vtk_polydata;
    vtkSmartPointer<vtkVertexGlyphFilter> vtk_glyphFilter;

public:
    Points(const int point_size)
    {
        // vtk_points_eigen=std::make_shared<vtkPointsEigen>();
        // vtk_points = vtk_points_eigen;
        vtk_points=vtkSmartPointer<vtkPoints>::New();

        vtk_polydata=vtkSmartPointer<vtkPolyData>::New();
        // vtk_polydata->SetPoints(vtk_points.get());
        vtk_polydata->SetPoints(vtk_points);

        vtk_glyphFilter=vtkSmartPointer<vtkVertexGlyphFilter>::New();
        vtk_glyphFilter->SetInputData(vtk_polydata);
        vtk_glyphFilter->Update();

        vtk_mapper=vtkSmartPointer<vtkPolyDataMapper>::New();
        vtk_mapper->SetInputConnection(vtk_glyphFilter->GetOutputPort());

        vtk_actor=vtkSmartPointer<vtkActor>::New();
        vtk_actor->SetMapper(vtk_mapper);
        vtk_actor->GetProperty()->SetPointSize(point_size);
    }

    void set_points(const Eigen::Ref<const Eigen::MatrixXd>& points)
    {
        // vtk_points_eigen->set_data(points);
        vtk_points=vtkSmartPointer<vtkPoints>::New();

        for (size_t i=0; i<points.cols(); i++)
            vtk_points->InsertNextPoint(points(0, i), points(1, i), points(2, i));

        // vtk_polydata->SetPoints(vtk_points.get());
        vtk_polydata->SetPoints(vtk_points);
    }

    bool set_colors(const std::vector<std::pair<int, int>>& coordinates, const Eigen::Ref<const Eigen::VectorXi>& valid_coordinates, const yarp::sig::ImageOf<yarp::sig::PixelRgb>& rgb_image)
    {
        vtk_colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        vtk_colors->SetNumberOfComponents(3);

        for (std::size_t i = 0; i < coordinates.size(); i++)
        {
            if (valid_coordinates(i) == 0)
                continue;

            yarp::sig::PixelRgb pixel = rgb_image.pixel(coordinates[i].first, coordinates[i].second);
            std::vector<unsigned char> color = {pixel.r, pixel.g, pixel.b};

            vtk_colors->InsertNextTypedTuple(color.data());
        }

        vtk_polydata->GetPointData()->SetScalars(vtk_colors);

        return true;
    }

    vtkSmartPointer<vtkPolyData> &get_polydata()
    {
        return vtk_polydata;
    }
};


class Viewer
{
public:
    Viewer(const std::string port_prefix, yarp::os::ResourceFinder& rf);

    virtual ~Viewer();

    void updateView();

private:
    void set2DCoordinates(const std::size_t u_stride, const std::size_t v_stride);

    std::tuple<bool, Eigen::MatrixXd, Eigen::VectorXi> get3DPointCloud(const yarp::sig::ImageOf<yarp::sig::PixelFloat>& depth, const std::string eye_name, const float z_threshold = 1.0);

    void setDefaultDeprojectionMatrix(const std::string eye_name);

    std::pair<bool, Eigen::MatrixXd> readStateFromFile(const std::string& filename, const std::size_t num_fields);

    yarp::os::BufferedPort<yarp::sig::Vector> port_estimate_in_;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> port_image_in_;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat>> port_depth_in_;

    std::vector<std::pair<int, int>> coordinates_2d_;

    std::unique_ptr<Points> vtk_measurements_;

    const std::size_t cam_width_ = 320;

    const std::size_t cam_height_ = 240;

    vtkSmartPointer<vtkPLYReader> reader_;

    vtkSmartPointer<vtkPolyDataMapper> mapper_;

    vtkSmartPointer<vtkActor> mesh_actor_;

    vtkSmartPointer<vtkRenderer> renderer_;

    vtkSmartPointer<vtkRenderWindow> render_window_;

    vtkSmartPointer<vtkRenderWindowInteractor> render_window_interactor_;

    vtkSmartPointer<vtkAxesActor> axes_;

    vtkSmartPointer<vtkOrientationMarkerWidget> orientation_widget_;

    vtkSmartPointer<vtkCamera> camera_;

    vtkSmartPointer<vtkInteractorStyleSwitch> interactor_style_;

    GazeController gaze_;

    Eigen::MatrixXd default_deprojection_matrix_;

    float pc_left_z_threshold_;

    bool show_hand_;

    std::unique_ptr<VtkiCubHand> vtk_icub_hand_;
};


class vtkUpdate : public vtkCommand
{
public:
    static vtkUpdate* New()
    {
        return new vtkUpdate;
    }

    void setViewer(Viewer* viewer)
    {
        viewer_ = viewer;
    }

    void Execute(vtkObject *caller, unsigned long vtkNotUsed(eventId), void * vtkNotUsed(callData))
    {
        viewer_->updateView();

        vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::SafeDownCast(caller);
        iren->GetRenderWindow()->Render();
    }

private:
    Viewer* viewer_;
};

#endif /* VIEWER_H */

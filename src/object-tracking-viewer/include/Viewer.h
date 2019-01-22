#ifndef VIEWER_H
#define VIEWER_H

#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkCamera.h>
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

#include <SFM.h>

#include <Eigen/Dense>

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
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


class Points : public Object
{
protected:
    vtkSmartPointer<vtkPoints> vtk_points;
    vtkSmartPointer<vtkUnsignedCharArray> vtk_colors;
    vtkSmartPointer<vtkPolyData> vtk_polydata;
    vtkSmartPointer<vtkVertexGlyphFilter> vtk_glyphFilter;

public:
    Points(const Eigen::Ref<const Eigen::MatrixXd>&points, const int point_size)
    {
        vtk_points=vtkSmartPointer<vtkPoints>::New();
        for (size_t i=0; i<points.cols(); i++)
            vtk_points->InsertNextPoint(points(0, i), points(1, i), points(2, i));

        vtk_polydata=vtkSmartPointer<vtkPolyData>::New();
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
        vtk_points=vtkSmartPointer<vtkPoints>::New();
        for (size_t i=0; i<points.cols(); i++)
            vtk_points->InsertNextPoint(points(0, i), points(1, i), points(2, i));

        vtk_polydata->SetPoints(vtk_points);
    }

    bool set_color(const int r, const int g, const int b)
    {
        std::vector<unsigned char> color =  {static_cast<unsigned char>(r),
                                             static_cast<unsigned char>(g),
                                             static_cast<unsigned char>(b)};

        vtk_colors=vtkSmartPointer<vtkUnsignedCharArray>::New();
        vtk_colors->SetNumberOfComponents(3);

        for (size_t i=0; i<vtk_points->GetNumberOfPoints(); i++)
            vtk_colors->InsertNextTypedTuple(color.data());

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
    std::pair<bool, Eigen::MatrixXd> readStateFromFile(const std::string& filename, const std::size_t num_fields);

    std::pair<bool, std::vector<Eigen::MatrixXd>> readMeasurementsFromFile(const std::string& filename);

    yarp::os::BufferedPort<yarp::sig::Vector> port_estimate_in_;

    // std::vector<Eigen::MatrixXd> measurements_;

    // std::unique_ptr<Points> vtk_measurements;

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

    SFM sfm_;
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

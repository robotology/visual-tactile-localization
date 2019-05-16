/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef VIEWER_H
#define VIEWER_H

#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkCamera.h>
#include <vtkContourFilter.h>
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
#include <vtkSampleFunction.h>
#include <vtkSmartPointer.h>
#include <vtkSuperquadric.h>
#include <vtkVertexGlyphFilter.h>

#include <Eigen/Dense>

#include <Camera.h>
#include <iCubCamera.h>
#include <RealsenseCamera.h>

#include <VtkiCubHand.h>

#include <opencv2/opencv.hpp>

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/Port.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>

#include <memory>
#include <string>

#include <thrift/ViewerIDL.h>


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

    bool set_colors(const std::vector<std::pair<int, int>>& coordinates, const Eigen::Ref<const Eigen::VectorXi>& valid_coordinates, const cv::Mat& rgb_image)
    {
        vtk_colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        vtk_colors->SetNumberOfComponents(3);

        for (std::size_t i = 0; i < coordinates.size(); i++)
        {
            if (valid_coordinates(i) == 0)
                continue;

            cv::Vec3b cv_color = rgb_image.at<cv::Vec3b>(cv::Point(coordinates[i].first, coordinates[i].second));
            /* yarp::sig::PixelRgb pixel = rgb_image.pixel(coordinates[i].first, coordinates[i].second); */
            /* std::vector<unsigned char> color = {pixel.r, pixel.g, pixel.b}; */
            std::vector<unsigned char> color = {cv_color[0], cv_color[1], cv_color[2]};

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


class Viewer : public ViewerIDL
{
public:
    Viewer(const std::string port_prefix, yarp::os::ResourceFinder& rf);

    virtual ~Viewer();

    void updateView();

    bool use_superquadric(const double size_x, const double size_y, const double size_z, const double eps_1, const double eps_2, const double x, const double y, const double z, const double phi, const double theta, const double psi) override;

private:
    void set2DCoordinates(const std::size_t u_stride, const std::size_t v_stride);

    std::tuple<bool, Eigen::MatrixXd, Eigen::VectorXi> get3DPointCloud(const Eigen::MatrixXf& depth, const float z_threshold = 1.0);

    yarp::os::BufferedPort<yarp::sig::Vector> port_estimate_in_;

    yarp::os::BufferedPort<yarp::sig::Vector> port_ground_truth_in_;

    yarp::os::Port port_rpc_command_;

    yarp::os::Mutex mutex_rpc_;

    std::vector<std::pair<int, int>> coordinates_2d_;

    std::unique_ptr<Points> vtk_measurements_;

    vtkSmartPointer<vtkPLYReader> reader_;

    vtkSmartPointer<vtkPolyDataMapper> mapper_;

    vtkSmartPointer<vtkActor> mesh_actor_;

    vtkSmartPointer<vtkActor> mesh_actor_ground_truth_;

    vtkSmartPointer<vtkRenderer> renderer_;

    vtkSmartPointer<vtkRenderWindow> render_window_;

    vtkSmartPointer<vtkRenderWindowInteractor> render_window_interactor_;

    vtkSmartPointer<vtkAxesActor> axes_;

    vtkSmartPointer<vtkOrientationMarkerWidget> orientation_widget_;

    vtkSmartPointer<vtkCamera> vtk_camera_;

    vtkSmartPointer<vtkInteractorStyleSwitch> interactor_style_;

    /**
     * Superquadric visualization
     */
    bool use_superquadric_visualization_ = false;

    vtkSmartPointer<vtkSuperquadric> superquadric_;

    vtkSmartPointer<vtkSampleFunction> superquadric_sample_;

    vtkSmartPointer<vtkContourFilter> superquadric_contours_;

    vtkSmartPointer<vtkPolyDataMapper> superquadric_mapper_;

    std::unique_ptr<Camera> camera_;

    Eigen::MatrixXd deprojection_matrix_;

    float pc_left_z_threshold_;

    bool show_hand_;

    bool show_ground_truth_;

    std::unique_ptr<VtkiCubHand> vtk_icub_hand_;

    const std::string log_ID_ = "[VIEWER]";
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

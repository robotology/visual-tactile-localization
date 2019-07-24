/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef VIEWER_H
#define VIEWER_H

#include <vtkActor.h>
#include <vtkOBJReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkPointData.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkVertexGlyphFilter.h>

#include <Eigen/Dense>

#include <string>

class LeftRightArrowManualPlayback;

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

    /****************************************************************/
    void set_points(const Eigen::Ref<const Eigen::MatrixXd>& points)
    {
        vtk_points=vtkSmartPointer<vtkPoints>::New();
        for (size_t i=0; i<points.cols(); i++)
            vtk_points->InsertNextPoint(points(0, i), points(1, i), points(2, i));

        vtk_polydata->SetPoints(vtk_points);
    }

    /****************************************************************/
    bool set_color(const std::string color_name)
    {
        std::vector<unsigned char> color_red =  {255, 0, 0  };
        std::vector<unsigned char> color_blue = {0,   0, 255};

        vtk_colors=vtkSmartPointer<vtkUnsignedCharArray>::New();
        vtk_colors->SetNumberOfComponents(3);

        for (size_t i=0; i<vtk_points->GetNumberOfPoints(); i++)
        {
            if (color_name == "red")
                vtk_colors->InsertNextTypedTuple(color_red.data());
            else if (color_name == "blue")
                vtk_colors->InsertNextTypedTuple(color_blue.data());
            else
                return false;
        }

        vtk_polydata->GetPointData()->SetScalars(vtk_colors);

        return true;
    }

    /****************************************************************/
    vtkSmartPointer<vtkPolyData> &get_polydata()
    {
        return vtk_polydata;
    }
};

class Visualizer
{
public:
    Visualizer
    (
        const std::string mesh_filename,
        const std::string trajectory_filename,
        const std::string estimate_filename,
        const std::string prediction_filename,
        const std::string measurements_data_filename,
        const std::string use_ground_truth
    );

    void stepForward();

    void stepBackward();

private:
    std::pair<bool, Eigen::MatrixXd> readStateFromFile(const std::string& filename, const std::size_t num_fields);

    std::pair<bool, std::vector<Eigen::MatrixXd>> readMeasurementsFromFile(const std::string& filename);

    void updateView();

    Eigen::MatrixXd target_;

    Eigen::MatrixXd estimate_;

    Eigen::MatrixXd prediction_;

    std::vector<Eigen::MatrixXd> measurements_;

    std::unique_ptr<Points> vtk_measurements;

    vtkSmartPointer<vtkOBJReader> reader_;

    vtkSmartPointer<vtkPolyDataMapper> mapper_;

    vtkSmartPointer<vtkActor> mesh_actor_;

    vtkSmartPointer<vtkActor> estimate_actor_;

    vtkSmartPointer<vtkActor> prediction_actor_;

    vtkSmartPointer<vtkRenderer> renderer_;

    vtkSmartPointer<vtkRenderWindow> renderWindow_;

    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor_;

    vtkSmartPointer<LeftRightArrowManualPlayback> manualPlaybackCtrl_;

    int step_;

    int num_steps_;

    bool use_ground_truth_;
};

#endif /* VIEWER_H */

#ifndef VTKMESH_H
#define VTKMESH_H

#include <Eigen/Dense>

#include <vtkActor.h>
#include <vtkOBJReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>


class VtkMesh
{
public:
    VtkMesh(const std::string mesh_path);

    virtual ~VtkMesh();

    void addToRenderer(vtkRenderer& renderer);

    void setPose(const Eigen::Ref<const Eigen::VectorXd>& pose);

private:
    vtkSmartPointer<vtkOBJReader> reader_;

    vtkSmartPointer<vtkPolyDataMapper> mapper_;

    vtkSmartPointer<vtkActor> mesh_actor_;
};

#endif /* VTKMESH_H */

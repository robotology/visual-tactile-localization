#include <VtkMesh.h>

#include <vtkTransform.h>

using namespace Eigen;


VtkMesh::VtkMesh(const std::string mesh_path)
{
    // Initialize mesh reader
    reader_ = vtkSmartPointer<vtkOBJReader>::New();
    reader_->SetFileName(mesh_path.c_str());
    reader_->Update();

    // Connect to PolyDataMapper
    mapper_ = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper_->SetInputConnection(reader_->GetOutputPort());

    // Initialize actor representing the input mesh
    mesh_actor_ = vtkSmartPointer<vtkActor>::New();
    mesh_actor_->SetMapper(mapper_);
}


VtkMesh::~VtkMesh()
{ }


void VtkMesh::addToRenderer(vtkRenderer& renderer)
{
    renderer.AddActor(mesh_actor_);
}


void VtkMesh::setPose(const Ref<const VectorXd>& pose)
{
    // Create a transform
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();

    // Set translation
    transform->Translate(pose.head<3>().data());

    // Set rotation
    transform->RotateWXYZ(pose(6) * 180 / M_PI,
                          pose(3), pose(4), pose(5));
    // Apply transform
    mesh_actor_->SetUserTransform(transform);
}

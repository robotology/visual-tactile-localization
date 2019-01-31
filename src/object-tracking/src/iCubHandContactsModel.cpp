#include <iCubHandContactsModel.h>

using namespace Eigen;

iCubHandContactsModel::iCubHandContactsModel(std::unique_ptr<iCubArmModel> icub_arm, std::vector<std::string> used_fingers) :
    icub_arm_(std::move(icub_arm)),
    used_fingers_(used_fingers)
{
    // Retrieve icub hand mesh paths
    bool valid_paths;
    SICAD::ModelPathContainer meshes_paths;
    std::tie(valid_paths, meshes_paths) = icub_arm_->getMeshPaths();

    if (!valid_paths)
    {
        std::string err = "ICUBHANDCONTACTSMODEL::CTOR::ERROR\n\tError: cannot load iCub hand meshes paths.";
        throw(std::runtime_error(err));
    }

    // Load all the meshes
    for (auto path : meshes_paths)
    {
        if (isHandPartEnabled(path.first))
        {
            if (!loadMesh(path.first, path.second))
            {
                std::string err = "ICUBHANDCONTACTSMODEL::CTOR::ERROR\n\tError: cannot load iCub hand meshes.";
                throw(std::runtime_error(err));
            }

            yInfo() << log_ID_ << "Mesh of hand part " + path.first + " successfully loaded.";
        }
    }

    // Sample clouds on fingertips
    for (auto used_finger : used_fingers)
        sampleFingerTip(getFingerTipName(used_finger));
}


std::string iCubHandContactsModel::getFingerTipName(const std::string finger_name)
{
    std::string fingertip_name = finger_name;

    if (finger_name == "middle")
    {
        fingertip_name += "3";
    }
    else
    {
        fingertip_name += "4";
    }

    return fingertip_name;
}


bool iCubHandContactsModel::isHandPartEnabled(const std::string hand_part_name)
{
    bool found = false;

    for (auto used_finger : used_fingers_)
    {
        if (hand_part_name.find(used_finger) != string::npos)
        {
            found = true;
            break;
        }
    }

    return found;
}


bool iCubHandContactsModel::loadMesh(const std::string hand_part_name, const std::string mesh_path)
{
    MeshImporter importer(mesh_path);

    // Convert mesh using MeshImporter
    std::istringstream mesh_input;
    bool valid_mesh;

    std::tie(valid_mesh, mesh_input) = importer.getMesh("obj");

    if (!valid_mesh)
    {
        yError() << log_ID_<< "ICUBHANDCONTACTSMODEL::CTOR::ERROR\n\tError: cannot load mesh file for hand part " + hand_part_name + ".";

        return false;
    }

    // Allocate storage for the mesh
    simpleTriMesh& mesh = hand_meshes_[hand_part_name];

    // Open converted obj using vcg mesh importer
    OBJImportInfo info;
    int outcome;
    outcome = simpleTriMeshImporter::OpenStream(mesh, mesh_input, info);

    if(simpleTriMeshImporter::ErrorCritical(outcome))
    {
        yError() << log_ID_<<  "ICUBHANDCONTACTSMODEL::CTOR::ERROR\n\tError: cannot load mesh file " + mesh_path +
                               ". Error:" + std::string(simpleTriMeshImporter::ErrorMsg(outcome)) + ".";
        return false;
    }

    // Update bounding box
    vcg::tri::UpdateBounding<simpleTriMesh>::Box(mesh);

    // Update face normals
    if(mesh.fn > 0)
        vcg::tri::UpdateNormal<simpleTriMesh>::PerFace(mesh);

    // Update vertex normals
    if(mesh.vn > 0)
	vcg::tri::UpdateNormal<simpleTriMesh>::PerVertex(mesh);

    return true;
}

#include <fstream>
void iCubHandContactsModel::sampleFingerTip(const std::string fingertip_name)
{
    simpleTriMesh& mesh = hand_meshes_.at(fingertip_name);

    // Perform back-face culling in order to remove faces pointing towards
    // the negative local z-axis (i.e. negative approach direction of the fingertip).
    // This way it is possible to sample points only on the surface of the fingertip
    // that may experience contacts with the external environment.
    Vector3d normal(3);
    Vector3d vertex0(3);
    Vector3d observer(3);
    observer << 0.0, 0.0, 0.1;
    for (FaceIterator fi = mesh.face.begin(); fi != mesh.face.end(); fi++)
    {
        // Extract face normal
        const auto n = fi->cN();
        normal << n[0], n[1], n[2];

        // Extract position of first vertex of the face
        const auto v = fi->cP(0);
        vertex0 << v[0], v[1], v[2];

        // Eval vector from view point to first vertex
        VectorXd diff = vertex0 - observer;

        // Perform culling check
        if (diff.dot(normal) >= 0)
            // Remove face
            simpleTriMeshAllocator::DeleteFace(mesh, *fi);
    }
    // Perform face garbage collection after culling
    simpleTriMeshAllocator::CompactFaceVector(mesh);

    // Perform Disk Poisson Sampling

    // Some default parametrs as found in MeshLab
    std::size_t oversampling = 20;
    triMeshSurfSampler::PoissonDiskParam poiss_params;
    poiss_params.radiusVariance = 1;
    poiss_params.geodesicDistanceFlag = false;
    poiss_params.bestSampleChoiceFlag = true;
    poiss_params.bestSamplePoolSize = 10;

    // Estimate radius required to obtain disk poisson sampling
    // with the number_of_points points
    simpleTriMesh::ScalarType radius;
    radius = triMeshSurfSampler::ComputePoissonDiskRadius(mesh, fingertip_number_samples_);

    // Generate preliminar montecarlo sampling with uniform probability
    simpleTriMesh montecarlo_mesh;
    triMeshSampler mc_sampler(montecarlo_mesh);
    mc_sampler.qualitySampling=true;
    triMeshSurfSampler::Montecarlo(mesh,
				   mc_sampler,
				   fingertip_number_samples_ * oversampling);
    // Copy the bounding box from the original mesh
    montecarlo_mesh.bbox = mesh.bbox;

    // Generate disk poisson samples by pruning the montecarlo cloud
    simpleTriMesh poiss_mesh;
    triMeshSampler dp_sampler(poiss_mesh);
    triMeshSurfSampler::PoissonDiskPruning(dp_sampler,
					   montecarlo_mesh,
					   radius,
					   poiss_params);
    vcg::tri::UpdateBounding<simpleTriMesh>::Box(poiss_mesh);

    // Store the cloud
    std::size_t number_points = std::distance(poiss_mesh.vert.begin(), poiss_mesh.vert.end());
    VectorXd cloud(3 * number_points);
    std::size_t i = 0;
    for (VertexIterator vi = poiss_mesh.vert.begin(); vi != poiss_mesh.vert.end(); vi++)
    {
	// Extract the point
	const auto p = vi->cP();

        // Add to the cloud
        cloud.segment(i * 3, 3) << p[0], p[1], p[2];

        i++;
    }

    sampled_fingertips_[fingertip_name] = cloud;
}

/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ObjectMeshSampler.h>

using namespace Eigen;


ObjectMeshSampler::~ObjectMeshSampler()
{ }


ObjectMeshSampler::ObjectMeshSampler(const std::string& mesh_filename) :
    MeshImporter(mesh_filename)
{
    // Convert mesh using MeshImporter
    std::istringstream mesh_input;
    bool valid_mesh;

    std::tie(valid_mesh, mesh_input) = getMesh("obj");

    if (!valid_mesh)
    {
        std::string err = log_ID_ + "::ctor Error: cannot load mesh file " + mesh_filename + ".";
        throw(std::runtime_error(err));
    }

    // Open converted obj using vcg mesh importer
    OBJImportInfo info;
    int outcome;
    outcome = simpleTriMeshImporter::OpenStream(trimesh_, mesh_input, info);

    if(simpleTriMeshImporter::ErrorCritical(outcome))
    {
        std::string err = log_ID_ + "::ctor Error: cannot load mesh file " + mesh_filename +
                          ". Error:" + std::string(simpleTriMeshImporter::ErrorMsg(outcome)) + ".";
        throw(std::runtime_error(err));
    }

    // Update bounding box
    vcg::tri::UpdateBounding<simpleTriMesh>::Box(trimesh_);

    // Update face normals
    if(trimesh_.fn > 0)
        vcg::tri::UpdateNormal<simpleTriMesh>::PerFace(trimesh_);

    // Update vertex normals
    if(trimesh_.vn > 0)
	vcg::tri::UpdateNormal<simpleTriMesh>::PerVertex(trimesh_);
}


std::pair<bool, MatrixXd> ObjectMeshSampler::sample(const std::size_t& number_of_points)
{
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
    radius = triMeshSurfSampler::ComputePoissonDiskRadius(trimesh_, number_of_points);

    // Generate preliminar montecarlo sampling with uniform probability
    simpleTriMesh montecarlo_mesh;
    triMeshSampler mc_sampler(montecarlo_mesh);
    mc_sampler.qualitySampling=true;
    triMeshSurfSampler::Montecarlo(trimesh_,
				   mc_sampler,
				   number_of_points * oversampling);
    // Copy the bounding box from the original mesh
    montecarlo_mesh.bbox = trimesh_.bbox;

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
    MatrixXd cloud(3, number_points);
    std::size_t i = 0;
    for (VertexIterator vi = poiss_mesh.vert.begin(); vi != poiss_mesh.vert.end(); vi++)
    {
        // Extract the point
        const auto p = vi->cP();

        // Add to the cloud
        cloud.col(i) << p[0], p[1], p[2];

        i++;
    }

    return std::make_pair(true, cloud);
}

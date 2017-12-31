/******************************************************************************
 *                                                                            *
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga <nicolapiga@gmail.com>
 */
#include "headers/fakePointCloud.h"

// yarp
#include <yarp/os/all.h>
#include <yarp/math/Math.h>

// mesh IO
#include <wrap/io_trimesh/import_off.h>
#include <wrap/io_trimesh/export_off.h>

using namespace yarp::math;

// mesh IO
typedef vcg::tri::io::ImporterOFF<simpleTriMesh> simpleTriMeshImporter;
typedef simpleTriMeshImporter::OFFCodes OFFImportErrors;

// mesh copy
typedef vcg::tri::Append<simpleTriMesh, simpleTriMesh> copyTriMesh;

// VCG vector and matrices
typedef vcg::Matrix44<simpleTriMesh::ScalarType> vcgHomMatrix;
typedef vcg::Point3<simpleTriMesh::ScalarType> vcgVector;

// VCG transformation
typedef vcg::tri::UpdatePosition<simpleTriMesh> transformTriMesh;

// VCG vertex
typedef simpleTriMesh::VertexIterator  VertexIterator;

bool FakePointCloud::loadObjectModel(const std::string &file_path)
{
    int outcome;
    outcome = simpleTriMeshImporter::Open(mesh, file_path.c_str());

    if(outcome != OFFImportErrors::NoError)
    {	
	yError() << "Error while importing .OFF file" << file_path;

	return false;
    }

    // update bounding box
    vcg::tri::UpdateBounding<simpleTriMesh>::Box(mesh);

    // update vertex normals
    vcg::tri::UpdateNormal<simpleTriMesh>::PerVertex(mesh);        

    // update face normals
    if(mesh.fn>0)
    	vcg::tri::UpdateNormal<simpleTriMesh>::PerFace(mesh);

    return true;
}

void FakePointCloud::setPose(const yarp::sig::Vector &pose)
{
    pos = pose.subVector(0,2);
    att = pose.subVector(3,5);
}

void FakePointCloud::transformModel(simpleTriMesh &mesh_out)
{
    // duplicate the original mesh
    copyTriMesh::Mesh(mesh_out, mesh);

    // evaluate rototranslation matrix
    vcgHomMatrix Htransl;
    Htransl.SetTranslate(pos[0], pos[1], pos[2]);

    vcgHomMatrix Hrot;
    yarp::sig::Matrix dcm = yarp::math::euler2dcm(att);
    yarp::sig::Vector axis_angle = yarp::math::dcm2axis(dcm);
    vcgVector axis(axis_angle[0], axis_angle[1], axis_angle[2]);
    Hrot.SetRotateRad(axis_angle[3], axis);

    vcgHomMatrix H = Htransl * Hrot;

    // apply rototranslation to the mesh    
    transformTriMesh::Matrix(mesh_out, H, true);
}

void FakePointCloud::samplePointCloud(std::vector<Point> &cloud,
				      const yarp::sig::Vector &obs_origin,
				      const int &num_points)
{
    // transform the model using the current pose set
    simpleTriMesh mesh_cp;
    transformModel(mesh_cp);
    
    // perform Poisson Disk Sampling
    
    // some default parametrs as found in MeshLab
    int oversampling = 20;
    triMeshSurfSampler::PoissonDiskParam poiss_params;
    poiss_params.radiusVariance = 1;
    poiss_params.geodesicDistanceFlag=false;
    poiss_params.bestSampleChoiceFlag=true;
    poiss_params.bestSamplePoolSize=10;
    
    // estimate radius required to obtain disk poisson sampling
    // with the given number of points
    simpleTriMesh::ScalarType radius;
    radius = triMeshSurfSampler::ComputePoissonDiskRadius(mesh_cp,
							  num_points);

    // generate preliminar montecarlo sampling with uniform probability
    simpleTriMesh montecarlo_mesh;
    triMeshSampler mc_sampler(montecarlo_mesh);
    mc_sampler.qualitySampling=true;
    triMeshSurfSampler::Montecarlo(mesh_cp,
				   mc_sampler,
				   num_points * oversampling);
    montecarlo_mesh.bbox = mesh.bbox;

    // generate poisson disk samples by pruning the montecarlo cloud
    simpleTriMesh poiss_mesh;
    triMeshSampler pd_sampler(poiss_mesh);
    triMeshSurfSampler::PoissonDiskPruning(pd_sampler,
					   montecarlo_mesh,
					   radius,
					   poiss_params);
    vcg::tri::UpdateBounding<simpleTriMesh>::Box(poiss_mesh);

    // store the vertices in the cloud
    for (VertexIterator vi = poiss_mesh.vert.begin();
	 vi != poiss_mesh.vert.end();
	 vi++)
    {
	// extract the point
	const auto p = vi->cP();
	yarp::sig::Vector point(3, 0.0);
	point[0] = p[0];
	point[1] = p[1];
	point[2] = p[2];

	// extract the associated normal
	const auto n = vi->cN();
	yarp::sig::Vector normal(3, 0.0);
	normal[0] = n[0];
	normal[1] = n[1];
	normal[2] = n[2];

	// eval vector from observer to point
	yarp::sig::Vector diff = point - obs_origin;

	// evaluate angle between diff and
	// the normal at the point considered
	double angle = acos(yarp::math::dot(diff, normal) /
	                    yarp::math::norm(diff) /
	                    yarp::math::norm(normal));
	
	// take the point if the angle is greater than 90 degrees
	if(angle > M_PI/2.0)
	    cloud.push_back(Point(p[0], p[1], p[2]));
    }
}

void FakePointCloud::getPointCloud(std::vector<Point> &cloud)
{
    // transform the model using the current pose set
    simpleTriMesh mesh_cp;
    transformModel(mesh_cp);

    // store the vertices in the cloud
    for (VertexIterator vi = mesh_cp.vert.begin();
	 vi != mesh_cp.vert.end();
	 vi++)
    {
	const vcg::Point3f p = vi->cP();
	cloud.push_back(Point(p[0], p[1], p[2]));
    }
}

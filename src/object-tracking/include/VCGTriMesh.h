/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef VCGTRIMESH_H
#define VCGTRIMESH_H

#include <vcg/complex/complex.h>
#include <vcg/complex/allocate.h>
#include <vcg/complex/algorithms/pointcloud_normal.h>
#include <vcg/complex/algorithms/point_sampling.h>
#include <vcg/complex/algorithms/update/position.h>

class vertex;
class edge;
class face;

struct usedTypes : public vcg::UsedTypes<vcg::Use<vertex> ::AsVertexType,
					 vcg::Use<edge>   ::AsEdgeType,
					 vcg::Use<face>   ::AsFaceType> { };

class vertex : public vcg::Vertex<usedTypes,
				  vcg::vertex::Coord3d,
				  vcg::vertex::Normal3d,
				  vcg::vertex::BitFlags> {};

class face : public vcg::Face<usedTypes,
			      vcg::face::VertexRef,
			      vcg::face::Normal3d,
			      vcg::face::BitFlags> {};

class edge : public vcg::Edge<usedTypes> {};

class simpleTriMesh : public vcg::tri::TriMesh<std::vector<vertex>,
					       std::vector<face>,
					       std::vector<edge> > {};

class triMeshSampler : public MeshSampler<simpleTriMesh>
{
public:
    triMeshSampler(simpleTriMesh &m) : MeshSampler<simpleTriMesh>(m) {}
    bool qualitySampling;
};

// mesh sampler
using triMeshSurfSampler = vcg::tri::SurfaceSampling<simpleTriMesh,triMeshSampler>;

// mesh IO
/* #include <wrap/io_trimesh/import_off.h> */
/* #include <wrap/io_trimesh/import_obj.h> */
#include <vcg_import_obj_w_stream.h>

// mesh IO
using simpleTriMeshImporter = vcg::tri::io::ImporterOBJWStream<simpleTriMesh>;
/* using simpleTriMeshImporter = vcg::tri::io::ImporterOBJ<simpleTriMesh>; */
/* using simpleTriMeshImporter = vcg::tri::io::ImporterOFF<simpleTriMesh> ; */
/* using OFFImportErrors = simpleTriMeshImporter::OFFCodes; */
using OBJImportErrors = simpleTriMeshImporter::OBJError;
using OBJImportInfo = simpleTriMeshImporter::Info;

// mesh copy
using copyTriMesh = vcg::tri::Append<simpleTriMesh, simpleTriMesh>;

// VCG vector and matrices
using vcgHomMatrix = vcg::Matrix44<simpleTriMesh::ScalarType>;
using vcgVector = vcg::Point3<simpleTriMesh::ScalarType>;
using vcgQuaternion = vcg::Quaternion<simpleTriMesh::ScalarType>;

// VCG transformation
using transformTriMesh = vcg::tri::UpdatePosition<simpleTriMesh>;

// VCG face
using FaceIterator = simpleTriMesh::FaceIterator;

// VCG vertex
using VertexIterator = simpleTriMesh::VertexIterator;

// VCG allocator
using simpleTriMeshAllocator = vcg::tri::Allocator<simpleTriMesh>;

// VCG point cloud normal estimation
using simpleTriMeshPointCloudNormal = vcg::tri::PointCloudNormal<simpleTriMesh>;

#endif /* VCGTRIMESH_H */

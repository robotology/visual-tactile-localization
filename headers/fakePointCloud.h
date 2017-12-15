/******************************************************************************
 *                                                                            *
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga <nicolapiga@gmail.com>
 */

#ifndef FAKE_POINT_CLOUD_H
#define FAKE_POINT_CLOUD_H

// VCG
#include <vcg/complex/complex.h>
#include <vcg/complex/allocate.h>
#include <vcg/complex/algorithms/point_sampling.h>
#include <vcg/complex/algorithms/update/position.h>

// yarp
#include <yarp/sig/all.h>

// CGAL
#include "geometryCGAL.h"

class vertex;
class edge;
class face;

struct usedTypes : public vcg::UsedTypes<vcg::Use<vertex> ::AsVertexType,
					 vcg::Use<edge>   ::AsEdgeType,
					 vcg::Use<face>   ::AsFaceType> { };

class vertex : public vcg::Vertex<usedTypes,
				  vcg::vertex::Coord3f,
				  vcg::vertex::Normal3f,
				  vcg::vertex::BitFlags> {};

class face : public vcg::Face<usedTypes,
			      vcg::face::VertexRef,
			      vcg::face::Normal3f,
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

typedef vcg::tri::SurfaceSampling<simpleTriMesh,
				  triMeshSampler> triMeshSurfSampler;
typedef vcg::Matrix44<simpleTriMesh::ScalarType> homogMatrix;

class FakePointCloud
{
private:
    // triangular mesh object
    simpleTriMesh mesh;

    // current position of the center of the object
    yarp::sig::Vector pos;

    // current attitude of the object
    yarp::sig::Vector att;
    
public:
    /*
     * Constructor.
     */
    FakePointCloud() : pos(3, 0.0), att(3, 0.0) { };
    
    /*
     * Load the model of the object as a triangular mesh
     * stored in a .OFF (Object File Format) file
     * The file could also be a Vertex Only .OFF conltaining a point cloud.
     * @param file_path the path of the file containing the model
     * @return true on success 
     */
    bool loadObjectModel(const std::string &file_path);

    /*
     * Set the current pose of the object.
     * @param pose yarp::sig::Vector containing the pose
     * as a position vector and three Euler ZYZ angles
     * @return true on success 
     */
    void setPose(const yarp::sig::Vector &pose);

    /*
     * Transform the model using the current pose set.
     * @param mesh_out a simpleTriMesh containing the transformed model
     */
    void transformModel(simpleTriMesh &mesh_out);

    /*
     * Sample a point cloud obtained by Disk Poismson sampling
     * the object in the current pose set.
     * @param cloud a std::vector<Point> of Point points
     * @param num_points the "approximate" number of points requested
     * containing the cloud
     * @return true on success 
     */
    void samplePointCloud(std::vector<Point> &cloud,
			  const int &num_points);
    /*
     * Get the original point cloud representing the loaded model
     * after its rototranslation using the current pose set.
     * @param cloud a std::vector<Point> of Point points
     * containing the cloud
     * @return true on success 
     */
    void getPointCloud(std::vector<Point> &cloud);
};

#endif

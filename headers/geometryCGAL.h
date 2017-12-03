/******************************************************************************
 *                                                                            *
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Giulia Vezzani <giulia.vezzani@iit.it>
 * @author: Nicola Piga <nicolapiga@gmail.com>
 */

#ifndef GEOMETRY_CGAL_H
#define GEOMETRY_CGAL_H

// CGAL
#include <CGAL/Simple_cartesian.h>
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>

typedef CGAL::Simple_cartesian<double> K;
typedef K::Point_3 Point;
typedef CGAL::Polyhedron_3<K> Polyhedron;
typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron> Primitive;
typedef CGAL::AABB_traits<K,Primitive> Traits;
typedef CGAL::AABB_tree<Traits> Tree;

class GeometryCGAL
{
protected:
    Polyhedron         model;
    Tree               tree;

public:
    Polyhedron &getModel() { return model; }

    void init()
    {
        // constructs AABB tree and computes internal KD-tree 
        // data structure to accelerate distance queries
        tree.insert(faces(model).first, faces(model).second, model);
        tree.accelerate_distance_queries();
    }
};

#endif

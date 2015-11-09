
/*
 * Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Giulia Vezzani
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/
#ifndef OPTIMIZER_H
#define OPTIMIZER_H
#include <deque>
#include <yarp/sig/all.h>

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


/*******************************************************************************/
struct Parameters
{
    yarp::sig::Vector x_lim;
    yarp::sig::Vector y_lim;
    yarp::sig::Vector z_lim;
    
    Parameters() : x_lim(3,0.0), y_lim(3,0.0), z_lim(3,0.0) { }
};


/*******************************************************************************/
class GeometryCGAL
{
protected:
    std::deque<Point>  measurements;
    Polyhedron         model;
    Tree               tree;
    Parameters        *parameters;  // owned by this superclass

public:
    /***************************************************************************/
    GeometryCGAL() : parameters(NULL) { }
    
    /***************************************************************************/
    std::deque<Point> &get_measurements() { return measurements; }
    
    /***************************************************************************/
    Polyhedron &get_model() { return model; }
    
    /***************************************************************************/
    virtual Parameters &get_parameters() { return *parameters; }
    
    /***************************************************************************/
    virtual void init()
    {
        // constructs AABB tree and computes internal KD-tree 
        // data structure to accelerate distance queries
        tree.insert(faces(model).first,faces(model).second,model);
        tree.accelerate_distance_queries();
    }
    
    /***************************************************************************/
    virtual bool step() = 0;

    /***************************************************************************/
    virtual yarp::sig::Vector finalize() = 0;
    
    /***************************************************************************/
    virtual ~GeometryCGAL()
    {
        delete parameters;
    }
};

#endif

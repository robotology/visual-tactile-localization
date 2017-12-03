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

#ifndef LOCALIZER_H
#define LOCALIZER_H

#include "headers/unscentedParticleFilter.h"
#include "headers/geometryCGAL.h"

/** 
 *  Results of the algorithm.
 */
struct Results
{
    // estimated pose    
    yarp::sig::Vector estimate;

    // performance index
    double perf_index;

    // final time without MAP estimate
    double tf_no_map;

    // final time with MAP estimate
    double tf;

    // initialization
    Results() : estimate(6,0.0),
	        tf_no_map(0),
	        tf(0) { }
};

/** 
 *  This class is the implementation of the Memory Unscented Particle Filter (MUPF)
 *  that solves the 6D localization problem offline.
 */
class Localizer : public yarp::os::RFModule
{
private:

    // resource finder
    yarp::os::ResourceFinder *rf;
    
    // pointer to Unscented Particle Filter
    UnscentedParticleFilter *upf;

    // measurements
    std::deque<Point> measurements;

    // results
    std::vector<Results> results;

    // number of trials
    int num_trials;

    // fixed number of contact points per time step
    int n_contacts;

    // real pose
    yarp::sig::Vector real_pose;
    
   /*
    * Load a measurements file. 
    * The file path is taken using the tag "measurementsFile" from a
    * previously instantiated @see Resource Finder.
    * @param rf a previously instantiated @see Resource Finder
    * @param meas a std::vector<Point> vector of Point points
    * @return true/false depending on the outcome
    */
    bool loadMeasurements(const yarp::os::ResourceFinder &rf);

    /*
     * Load the require parameters using a
     * previously instantiated @see Resource Finder.
     * @param rf a previously instantiated @see Resource Finder
     * @return true/false depending on the outcome
     */
    bool loadParameters(const yarp::os::ResourceFinder &rf);

    /*
     * Save a Polyhedron model as a .OFF (Object File Format) file.
     *
     * @param model a Polyhedron model
     * @param filename the output file name
     * @return true/false depending on the outcome
     */
    bool saveModel(const Polyhedron &model, const std::string &filename);    

    /**
     * Save the performance error index, the execution time 
     * and the solution found for all the trials.
     * @param results the results to be saved
     * @return true/false depending on the outcome
     */
    bool saveResults(const std::vector<Results> &results);
    
public:
    /*
     * Configure the module.
     * @param rf a previously instantiated @see ResourceFinder
     */
    bool configure(yarp::os::ResourceFinder &rf) override;
    
    /*
     * Define the behavior of this module.
     */
    bool updateModule() override;

    /*
     * Define the cleanup behavior.
     */
    bool close() override;    
};

#endif

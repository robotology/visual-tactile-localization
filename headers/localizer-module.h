/******************************************************************************
 *                                                                            *
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga <nicolapiga@gmail.com>
 */

#ifndef LOCALIZER_MODULE_H
#define LOCALIZER_MODULE_H

// yarp
#include <yarp/os/BufferedPort.h>

// std
#include <vector>

#include "headers/filterData.h"
#include "headers/unscentedParticleFilter.h"

/** 
 *  Results of the algorithm.
 */
struct Results
{
    // real pose
    yarp::sig::Vector real;

    // estimated pose    
    yarp::sig::Vector estimate;

    // execution time of filtering step
    double exec_time;

    // type of step, i.e. vision or tactile
    int step_type;

    Results() : real(6, 0.0),
	        estimate(6, 0.0) {};
};

/** 
 *  This class is the implementation of an Online UPF based localizer algorithm.
 */
class LocalizerModule : public yarp::os::RFModule
{
private:
    // resource finder
    yarp::os::ResourceFinder *rf;

    // pointer to Unscented Particle Filter
    UnscentedParticleFilter upf;

    // storage for the new measure
    std::vector<yarp::sig::Vector> meas;

    // storage for the new input
    std::vector<yarp::sig::Vector> input;

    // system noise covariance matrices
    yarp::sig::Vector Q_vision;
    yarp::sig::Vector Q_tactile;

    // last estimate available
    yarp::sig::Vector last_estimate;

    // number of steps performed
    int n_steps;

    // results
    std::vector<Results> results;

    // port where new data is received
    yarp::os::BufferedPort<yarp::sig::FilterData> port_in;

    /*
     * Load the required parameters using a
     * previously instantiated @see Resource Finder.
     * @param rf a previously instantiated @see Resource Finder
     * @return true/false depending on the outcome
     */
    bool loadParameters(const yarp::os::ResourceFinder &rf);

    /*
     * Perform a filtering step using new data
     * @param data yarp::sig::FilterData data
     */
    void performFiltering(const yarp::sig::FilterData &data);

public:
    LocalizerModule() : last_estimate(6, 0.0),
            	        Q_vision(6, 0.0),
	                Q_tactile(6, 0.0) { };
    /*
     * Configure the module.
     * @param rf a previously instantiated @see ResourceFinder
     */
    bool configure(yarp::os::ResourceFinder &rf) override;

    /*
     * Return the module period.
     */
    double getPeriod() override;
    
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

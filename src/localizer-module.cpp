/******************************************************************************
 *                                                                            *
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga <nicolapiga@gmail.com>
 */

// yarp
#include <yarp/os/all.h>
#include <yarp/math/Math.h>
#include <yarp/os/Vocab.h>

//
#include "headers/localizer-module.h"
#include "headers/filterData.h"
#include "headers/unscentedParticleFilter.h"

using namespace yarp::math;

bool LocalizerModule::loadParameters(const yarp::os::ResourceFinder &rf)
{
    return true;
}

void LocalizerModule::performFiltering(const yarp::sig::FilterData &data)
{
    // extract tag,
    // i.e. VIS for data from vision or
    // TAC for tactile data
    int tag = data.tag();

    // extract the measure
    data.points(meas);

    // extract the input
    data.inputs(input);

    // set the appropriate noise covariance matrix
    switch(tag)
    {
    case VOCAB3('V','I','S'):
	upf.setQ(Q_vision);
	break;
    case VOCAB3('T','A','C'):
	upf.setQ(Q_tactile);
	break;
    }

    // set real pose
    // upf.setRealPose(pose);
    
    // set input
    upf.setNewInput(input[0]);
    
    // set new measure
    upf.setNewMeasure(meas);

    // step and estimate
    // t_i = yarp::os::SystemClock::nowSystem();
    upf.step();
    last_estimate = upf.getEstimate();
    // t_f = yarp::os::SystemClock::nowSystem();
    // diff = t_f - t_i;
}

bool LocalizerModule::configure(yarp::os::ResourceFinder &rf)
{
    // save reference to resource finder
    this->rf = &rf;

    // open the port
    // TODO: take port name from configuration file
    port_in.open("/upf-localizer:i");

    // initialize system noise covariance matrices
    // TODO: take these from the configuration file
    Q_vision.setSubvector(0, yarp::sig::Vector(3, 0.0001));
    Q_vision.setSubvector(3, yarp::sig::Vector(3, 0.01));		
    
    Q_tactile[0] = 0.00001;
    Q_tactile[1] = 0.00001;	
    Q_tactile[2] = 0.00000001;
    Q_tactile[3] = 0.01;
    Q_tactile[4] = 0.000001;
    Q_tactile[5] = 0.000001;	
    
    // configure and init the UPF
    if(!upf.configure(rf))
    	return false;
    upf.init();

    // reset the number of steps performed
    n_steps = 0;

    return true;
}

double LocalizerModule::getPeriod()
{
    return 0.05;
}

bool LocalizerModule::updateModule()
{
    // check if the module should stop
    if (isStopping())
	return false;

    // try to read data from the port
    bool should_wait = false;
    yarp::sig::FilterData *data = port_in.read(should_wait);

    // if new data available perform a filtering step
    if (data != YARP_NULLPTR)
	performFiltering(*data);

    return true;
}

bool LocalizerModule::close()
{
    // close the port
    port_in.close();
}

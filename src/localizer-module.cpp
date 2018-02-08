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
#include <yarp/math/FrameTransform.h>

//
#include "headers/localizer-module.h"
#include "headers/filterData.h"
#include "headers/unscentedParticleFilter.h"

using namespace yarp::math;

bool LocalizerModule::loadParameters(const yarp::os::ResourceFinder &rf)
{
    return true;
}

bool LocalizerModule::performFiltering(const yarp::sig::FilterData &data)
{
    // increment number of steps performed
    n_steps++;
    
    // extract tag,
    // i.e. VIS for data from vision or
    // TAC for tactile data
    int tag = data.tag();

    // extract the measure
    data.points(meas);
    if (meas.size() == 0)
	return false;

    // extract the input
    data.inputs(input);
    if (input.size() == 0)
	return false;

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

    return true;
}

void LocalizerModule::publishEstimate()
{    
    // convert the estimated pose to a homogeneous transformation matrix
    yarp::math::FrameTransform pose;
    pose.transFromVec(last_estimate[0],
		      last_estimate[1],
		      last_estimate[2]);

    // estimate is saved as x-y-z-Y-P-R
    pose.rotFromRPY(last_estimate[5],
		    last_estimate[4],
		    last_estimate[3]);
    
    // Set a new transform
    // TODO: read source and target from the configuration file
    tfClient->setTransform("/mustard/estimate/frame",
			   "/iCub/frame",
			   pose.toMatrix());
}

bool LocalizerModule::configure(yarp::os::ResourceFinder &rf)
{
    // save reference to resource finder
    this->rf = &rf;

    // open the port
    // TODO: take port name from configuration file
    bool ok_port = port_in.open("/upf-localizer:i");

    // stop the configuration if the port open failed
    if (!ok_port)
    {
	yError() << "LocalizerModule::Configure error:"
	         << "failure in opening input port /upf-localizer:i";
	return false;
    }

    // set FIFO policy
    port_in.setStrict();

    // prepare properties for the PolyDriver
    yarp::os::Property propTfClient;    
    propTfClient.put("device", "transformClient");
    propTfClient.put("local", "/upf-localizer/transformClient");
    propTfClient.put("remote", "/transformServer");
    propTfClient.put("period", getPeriod() * 1000);

    // open the driver and obtain a a IFrameTransform view
    tfClient = nullptr;    
    bool ok_drv = drvTransformClient.open(propTfClient);
    ok_drv = ok_drv && drvTransformClient.view(tfClient) && tfClient != nullptr;
    
    // stop configuration if the driver open failed
    // or the view retrieval failed
    // or the IFrameTransform pointer is not valid
    if (!ok_drv)
    {
	yError() << "LocalizerModule::Configure error:"
	         << "failure in opening iFrameTransform interface";
	return false;
    }

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
    // TODO: take from the config
    return 0.02;
}

bool LocalizerModule::updateModule()
{
    // check if the module should stop
    if (isStopping())
	return false;

    // try to read data from the port
    bool should_wait = false;
    data = port_in.read(should_wait);
        
    // if new data available perform a filtering step
    if (data != YARP_NULLPTR)
	if (!performFiltering(*data))
	{
	    yError() << "LocalizerModule: Error while performing filtering step";
	    return false;
	}

    // publish the last estimate available
    if (n_steps > 0)
	publishEstimate();

    return true;
}

bool LocalizerModule::close()
{
    // close the port
    port_in.close();
}

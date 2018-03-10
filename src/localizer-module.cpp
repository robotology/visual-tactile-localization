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

bool LocalizerModule::loadParameters()
{
    source_frame_name = rf->find("sourceFrame").asString();
    if (rf->find("sourceFrame").isNull())
	source_frame_name = "/iCub/frame";
    yInfo() << "Localizer module: source frame name is" << source_frame_name;

    target_frame_name = rf->find("targetFrame").asString();
    if (rf->find("targetFrame").isNull())
	target_frame_name = "/estimate/frame";
    yInfo() << "Localizer module: target frame name is" << target_frame_name;

    input_port_name = rf->find("inputPort").asString();
    if (rf->find("inputPort").isNull())
	input_port_name = "/upf-localizer:i";
    yInfo() << "Localizer module: input port name is" << input_port_name;

    rpc_port_name = rf->find("rpcServerPort").asString();
    if (rf->find("rpcServerPort").isNull())
    	rpc_port_name = "/upf-localize/service";
    yInfo() << "Localizer module: rpc server port name is" << rpc_port_name;

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
    tf_client->setTransform(target_frame_name,
			   source_frame_name,
			   pose.toMatrix());
}

bool LocalizerModule::respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply)
{
    std::string cmd = command.get(0).asString();
    if (cmd == "help")
    {
	reply.addVocab(yarp::os::Vocab::encode("many"));
	reply.addString("Available commands:");
	reply.addString("- storage-on");
	reply.addString("- storage-off");
	reply.addString("- storage-save");
	reply.addString("- reset");
	reply.addString("- quit");
    }
    else if (cmd == "storage-on")
    {
    }
    else if (cmd == "storage-off")
    {
    }
    else if (cmd == "storage-save")
    {
    }
    else if (cmd == "reset")
    {
    }
    else
	// the father class already handles the "quit" command
	return RFModule::respond(command,reply);

    return true;
}

bool LocalizerModule::configure(yarp::os::ResourceFinder &rf)
{
    // save reference to resource finder
    this->rf = &rf;

    // load parameters from the configuration file
    loadParameters();

    // open the port
    bool ok_port = port_in.open(input_port_name);

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
    tf_client = nullptr;
    bool ok_drv = drv_transform_client.open(propTfClient);
    ok_drv = ok_drv && drv_transform_client.view(tf_client) && tf_client != nullptr;

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

    // start rpc server
    rpc_port.open(rpc_port_name);
    attach(rpc_port);

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

    // close the driver
    drv_transform_client.close();
}

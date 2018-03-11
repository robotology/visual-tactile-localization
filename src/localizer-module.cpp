
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

// CGAL
#include <CGAL/IO/Polyhedron_iostream.h>

// std
#include <fstream>

//
#include "headers/localizer-module.h"
#include "headers/filterData.h"
#include "headers/unscentedParticleFilter.h"
#include "headers/geometryCGAL.h"

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
    	rpc_port_name = "/upf-localizer/service";
    yInfo() << "Localizer module: rpc server port name is" << rpc_port_name;

    if (!rf->check("outputPath"))
    {
        yError() << "Localizer module: output path not provided!";
        return false;
    }
    output_path = rf->findFile("outputPath");
    yInfo() << "Localizer module: output path is" << output_path;

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
    // using time of simulated environment
    // in case env variable YARP_CLOCK is set
    t_i = yarp::os::Time::now();
    upf.step();
    last_estimate = upf.getEstimate();
    t_f = yarp::os::Time::now();
    exec_time = t_f - t_i;

    // if requested by the user store
    // the data associated to this filtering step
    storage_on_mutex.lock();

    if (storage_on)
	storeData(last_ground_truth,
		  last_estimate,
		  meas,
		  input[0],
		  exec_time);

    storage_on_mutex.unlock();

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

bool LocalizerModule::retrieveGroundTruth(yarp::sig::Vector &pose)
{
    // Get the pose of the root frame of the robot
    // TODO: get source and target from configuration file
    yarp::sig::Matrix inertialToRobot(4,4);
    std::string source = "/iCub/frame";
    std::string target = "/box_alt/frame";

    // Get the transform
    if (!tf_client->getTransform(target, source, inertialToRobot))
	return false;

    pose.resize(6);

    // Extract position and ZXY euler angles
    yarp::math::FrameTransform frame;
    frame.fromMatrix(inertialToRobot);

    yarp::math::FrameTransform::Translation_t &pos = frame.translation;
    pose[0] = pos.tX;
    pose[1] = pos.tY;
    pose[2] = pos.tZ;

    pose.setSubvector(3, frame.getRPYRot());

    return true;
}

void LocalizerModule::resetStorage()
{
    // reset internal storage
    storage.clear();
}

void LocalizerModule::storeData(const yarp::sig::Vector &ground_truth,
				const yarp::sig::Vector &estimate,
				const std::vector<yarp::sig::Vector> &meas,
				const yarp::sig::Vector &input,
				const double &exec_time)
{
    Data d;

    // populate
    d.ground_truth = ground_truth;
    d.estimate = estimate;
    d.meas = meas;
    d.input = input;
    d.exec_time = exec_time;

    // add to storage
    storage.push_back(d);
}

bool LocalizerModule::saveMesh(const yarp::sig::Vector &pose,
			       const std::string &file_name)
{
    // obtain the mesh from the filter
    // (the filter contains the model of the object)
    Polyhedron p;
    upf.transformObject(pose, p);

    // compose file name
    std::string file_path = output_path + file_name;

    // save the model
    // overwrite in case it already exists
    std::ofstream fout(file_path.c_str(), std::ios::trunc);
    if(fout.is_open())
    	fout << p;
    else
    {
	fout.close();

    	yError() << "LocalizerModule: unable to save mesh file to"
		 << file_path;
	return false;
    }

    fout.close();

    return true;
}

bool LocalizerModule::saveMeas(const std::vector<yarp::sig::Vector> &meas,
			       const std::string &file_name)
{
    // save the measures
    // overwrite if it already exists
    std::ofstream fout(file_name.c_str());
    if(fout.is_open())
    {
	// print the OFF header
	fout << "OFF" << std::endl;
	// this is a vertices only .OFF
	fout << meas.size() << " 0 0" << std::endl;

	// save all the measurements
	for (const yarp::sig::Vector &m : meas)
	{
	    fout << m[0] << " "
		 << m[1] << " "
		 << m[2] << " "
		 << std::endl;
	}
    }
    else
    {
	fout.close();

    	yError() << "LocalizerModule: problem opening meas output file"
		 << file_name;
	return false;
    }

    fout.close();

    return true;
}

bool LocalizerModule::saveData(const std::vector<Data> &data)
{
    // compose file name for report file
    std::string report_path = output_path + "report.csv";

    // save the data
    // overwrite in case it already exists
    std::ofstream fout(report_path.c_str(), std::ios::trunc);
    if(fout.is_open())
    {
	// print the CSV header
	fout << "step;"
	     << "x_real;" << "y_real;" << "z_real;"
	     << "phi_real;" << "theta_real;" << "psi_real;"
	     << "x_sol;"   << "y_sol;"     << "z_sol;"
	     << "phi_sol;" << "theta_sol;" << "psi_sol;"
	     << "input_x;"   << "input_y;" << "input_z;"
	     << "exec_time;"
	     << std::endl;

	// save data for each step
	int step_index = 0;
	for (const Data& d : data)
	{
	    // index
	    fout << step_index << ";";
	    // ground truth
	    for(size_t j=0; j<6; j++)
		fout << d.ground_truth[j] << ";";
	    // estimate
	    for(size_t j=0; j<6; j++)
		fout << d.estimate[j] << ";";
	    // input
	    for(size_t j=0; j<3; j++)
		fout << d.input[j] << ";";
	    // execution time
	    fout << d.exec_time << ";";

	    fout << std::endl;

	    // save measurements separately since
	    // their numbers change from step to step
	    std::string meas_path = output_path + "meas_step_"
		+ std::to_string(step_index) + ".off";
	    if (!saveMeas(d.meas, meas_path))
	    {
		// error message is provided by saveMeas()
		fout.close();
		return false;
	    }

	    // save mesh of ground truth pose
	    std::string gt_mesh_path = output_path + "gt_mesh_step_"
		+ std::to_string(step_index) + ".off";
	    if (!saveMesh(d.ground_truth, gt_mesh_path))
	    {
		// error message is provided by saveMesh()
		fout.close();
		return false;
	    }

	    // save mesh of estimated pose
	    std::string est_mesh_path = output_path + "est_mesh_step_"
		+ std::to_string(step_index) +  ".off";
	    if (!saveMesh(d.estimate, est_mesh_path))
	    {
		// error message is provided by saveMesh()
		fout.close();
		return false;
	    }

	    step_index++;
	}
    }
    else
    {
	fout.close();

    	yError() << "LocalizeModule: problem opening output report file"
		 << report_path;
	return false;
    }

    fout.close();

    return true;
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
	// change flag
	storage_on_mutex.lock();
	storage_on = true;
	storage_on_mutex.unlock();

	resetStorage();

	reply.addString("Storage enabled succesfully.");
    }
    else if (cmd == "storage-off")
    {
	// change flag
	storage_on_mutex.lock();
	storage_on = false;
	storage_on_mutex.unlock();

	reply.addString("Storage disabled succesfully.");
    }
    else if (cmd == "storage-save")
    {
	bool ok;

	ok = saveData(storage);
	if (ok)
	    reply.addString("Storage saved to file succesfully.");
	else
	    reply.addString("Storage save failed.");
    }
    else if (cmd == "reset")
    {
	// reset the filter
	upf.init();

	reply.addString("Filter reset successful.");
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
    if (!loadParameters())
    {
	yError() << "LocalizerModule::Configure error:"
	         << "failure in loading parameters from configuration file";
	return false;
    }

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

    // reset storage
    storage_on = false;
    resetStorage();

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

    // try to get the ground truth
    retrieveGroundTruth(last_ground_truth);

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

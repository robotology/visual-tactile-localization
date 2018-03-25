
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
#include <yarp/os/Bottle.h>

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

bool LocalizerModule::readDiagonalMatrix(const std::string &tag,
					 const int &size,
					 yarp::sig::Vector &diag)
{
    // read values of the diagonal
    yarp::os::Bottle *b = rf->find(tag.c_str()).asList();

    if (b == NULL)
	return false;

    if (b->size() < size)
	return false;

    diag.resize(size, 0.0);
    for(size_t i; i < size; i++)
	diag[i] = b->get(i).asDouble();

    return true;
}

bool LocalizerModule::loadParameters()
{
    est_source_frame_name = rf->find("estimateSourceFrame").asString();
    if (rf->find("estimateSourceFrame").isNull())
	est_source_frame_name = "/iCub/frame";
    yInfo() << "Localizer module: estimate source frame name is" << est_source_frame_name;

    est_target_frame_name = rf->find("estimateTargetFrame").asString();
    if (rf->find("estimateTargetFrame").isNull())
	est_target_frame_name = "/estimate/frame";
    yInfo() << "Localizer module: estimate target frame name is" << est_target_frame_name;

    robot_source_frame_name = rf->find("robotSourceFrame").asString();
    if (rf->find("robotSourceFrame").isNull())
	robot_source_frame_name = "/inertial";
    yInfo() << "Localizer module: robot source frame name is" << robot_source_frame_name;

    robot_target_frame_name = rf->find("robotTargetFrame").asString();
    if (rf->find("robotTargetFrame").isNull())
	robot_target_frame_name = "/iCub/frame";
    yInfo() << "Localizer module: robot target frame name is" << robot_target_frame_name;

    input_port_name = rf->find("inputPort").asString();
    if (rf->find("inputPort").isNull())
	input_port_name = "/upf-localizer:i";
    yInfo() << "Localizer module: input port name is" << input_port_name;

    rpc_port_name = rf->find("rpcServerPort").asString();
    if (rf->find("rpcServerPort").isNull())
    	rpc_port_name = "/upf-localizer/service";
    yInfo() << "Localizer module: rpc server port name is" << rpc_port_name;

    port_pc_name = rf->find("pointCloudInputPort").asString();
    if (rf->find("pointCloudInputPort").isNull())
	port_pc_name = "/upf-localizer/pc:i";
    yInfo() << "Localizer module: point cloud input port name is" << port_pc_name;

    if (!rf->check("outputPath"))
    {
        yError() << "Localizer module: output path not provided!";
        return false;
    }
    output_path = rf->findFile("outputPath");
    yInfo() << "Localizer module: output path is" << output_path;

    if (!readDiagonalMatrix("visionQ", 6, Q_vision))
    {
	// set default value for covariance matrix
	Q_vision.setSubvector(0, yarp::sig::Vector(3, 0.0001));
	Q_vision.setSubvector(3, yarp::sig::Vector(3, 0.01));
    }
    yInfo() << "Localizer module: Q matrix for vision is" << Q_vision.toString();

    if (!readDiagonalMatrix("tactileQ", 6, Q_tactile))
    {
	Q_tactile[0] = 0.00001;
	Q_tactile[1] = 0.00001;
	Q_tactile[2] = 0.00000001;
	Q_tactile[3] = 0.01;
	Q_tactile[4] = 0.000001;
	Q_tactile[5] = 0.000001;
    }
    yInfo() << "Localizer module: Q matrix for tactile is" << Q_tactile.toString();

    R_vision = rf->find("visionR").asDouble();
    if (rf->find("visionR").isNull())
	R_vision = 0.0001;
    yInfo() << "Localizer module: R for vision is" << R_vision;

    R_tactile = rf->find("tactileR").asDouble();
    if (rf->find("tactileR").isNull())
	R_tactile = 0.0001;
    yInfo() << "Localizer module: R for tactile is" << R_tactile;    

    return true;
}

void LocalizerModule::transformPointCloud(const PointCloud& pc,
					  std::vector<yarp::sig::Vector> &pc_out)
{
    // copy data to pc_out
    for (size_t i=0; i<pc.size(); i++)
    {
	PointCloudItem item = pc[i];
	yarp::sig::Vector point(3, 0.0);
	point[0] = item.x;
	point[1] = item.y;
	point[2] = item.z;

	pc_out.push_back(point);
    }

    // transform the points taking into account
    // the root link of the robot
    for (size_t i=0; i<pc_out.size(); i++)
    {
	yarp::sig::Vector point(4, 0.0);
	point.setSubvector(0, pc_out[i]);
	point[3] = 1;

	// transform the point so that
	// it is relative to the orign of the robot root frame
	// and expressed in the robot root frame
	point = SE3inv(inertial_to_robot) * point;

	pc_out[i] = point.subVector(0,2);
    }
}

void LocalizerModule::processCommand(const yarp::sig::FilterData &filter_cmd)
{
    // extract command and filtering type
    int cmd = filter_cmd.command();
    int type = filter_cmd.tag();

    if (cmd == VOCAB2('O','N'))
    {
	filtering_enabled = true;
	if (type == VOCAB3('V', 'I', 'S'))
	    filtering_type = FilteringType::visual;
	else if (type == VOCAB3('T','A','C'))
	    filtering_type = FilteringType::tactile;
    }
    else if (cmd == VOCAB3('O','F','F'))
    {
	stopFiltering();
    }
}

void LocalizerModule::performFiltering(const yarp::sig::FilterData &data)
{
    if (!filtering_enabled)
	return;

    // extract the measure
    data.points(meas);
    // if (meas.size() == 0)
    // 	return;

    // extract the input
    data.inputs(input);
    // if (input.size() == 0)
    // 	return;

    if (filtering_type == FilteringType::visual)
    {
	// check if a point cloud is available
	PointCloud *new_pc = port_pc.read(false);
	if (new_pc == NULL)
	{
	    // nothing to do here
	    return;
	}

	// transform point cloud
	std::vector<yarp::sig::Vector> pc;
	transformPointCloud(*new_pc,
			    pc);

	// set noise covariances
	upf.setQ(Q_vision);
	upf.setR(R_vision);

	// process cloud in chuncks of 10 points
        // TODO: take n_points from config
	int n_points = 10;
	for (size_t i=0; i+n_points <= pc.size(); i += n_points)
	{
            // prepare measure
	    std::vector<yarp::sig::Vector> measure;
	    for (size_t k=0; k<n_points; k++)
		measure.push_back(pc[i+k]);

	    // set measure
	    upf.setNewMeasure(measure);

            // set zero input (visual localization is static)
	    yarp::sig::Vector input(3, 0.0);
	    upf.setNewInput(input);

	    // step and estimate
	    // using time of simulated environment
	    // in case env variable YARP_CLOCK is set
	    t_i = yarp::os::Time::now();
	    upf.step();
	    last_estimate = upf.getEstimate();
	    t_f = yarp::os::Time::now();
	    exec_time = t_f - t_i;

	    storage_on_mutex.lock();

	    // store data if required
	    if (storage_on)
		storeData(last_ground_truth,
			  last_estimate,
			  measure,
			  input,
			  exec_time);

	    storage_on_mutex.unlock();

	    // after a visual filtering step
	    // the filter disables automatically
	    // TODO: make this optional from configuration
	    //       or from FilterCommand
	    stopFiltering();

	    // a new estimate is now available
	    estimate_available = true;
	}
    }
    else if (filtering_type == FilteringType::tactile)
    {
	// set noise covariances
	upf.setQ(Q_tactile);
	upf.setR(R_tactile);

	// set measure
	upf.setNewMeasure(meas);

	// set input
	upf.setNewInput(input[0]);

	if (is_first_step)
	{
	    upf.resetTime();
	    is_first_step = false;
	}

	t_i = yarp::os::Time::now();

	if (meas.size() < 2)
	    // skip step in case of too few measurements
	    upf.skipStep();
	else
	{
	    // step and estimate
	    upf.step();
	    last_estimate = upf.getEstimate();
	}

	t_f = yarp::os::Time::now();
	exec_time = t_f - t_i;

	storage_on_mutex.lock();

	// store data if required
	if (storage_on)
	    storeData(last_ground_truth,
		      last_estimate,
		      meas,
		      input[0],
		      exec_time);

	storage_on_mutex.unlock();

	// a new estimate is now available
	estimate_available = true;
    }
}

void LocalizerModule::stopFiltering()
{
    filtering_enabled = false;

    // reset flag for the next activation
    is_first_step = true;
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
    tf_client->setTransform(est_target_frame_name,
			   est_source_frame_name,
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

    yarp::sig::Vector angles = frame.getRPYRot();
    pose[3] = angles[2];
    pose[4] = angles[1];
    pose[5] = angles[0];

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

    // save the model
    // overwrite in case it already exists
    std::ofstream fout(file_name.c_str(), std::ios::trunc);
    if(fout.is_open())
    	fout << p;
    else
    {
	fout.close();

    	yError() << "LocalizerModule: unable to save mesh file to"
		 << file_name;
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

    // open ports
    bool ok_port = port_in.open(input_port_name);

    // stop the configuration if the port open failed
    if (!ok_port)
    {
	yError() << "LocalizerModule::Configure error:"
	         << "failure in opening input port /upf-localizer:i";
	return false;
    }

    ok_port = port_pc.open(port_pc_name);
    if (!ok_port)
    {
	yError() << "LocalizerModule:Configure error:"
		 << "unable to open the point cloud port";
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

    // get the pose of the root frame of the robot
    // required to convert point clouds
    inertial_to_robot.resize(4,4);
    bool ok = false;
    double t0 = yarp::os::SystemClock::nowSystem();
    while (yarp::os::SystemClock::nowSystem() - t0 < 10.0)
    {
	// this might fail if the gazebo pluging
	// publishing the pose is not started yet
	if (tf_client->getTransform(robot_target_frame_name,
				    robot_source_frame_name,
				    inertial_to_robot))
	{
	    ok = true;
	    break;
	}
	yarp::os::SystemClock::delaySystem(1.0);
    }
    if (!ok)
    {
	yError() << "LocalizerModule: unable to get the pose of the root frame of the robot";
	return false;
    }

    // prepare properties for the Encoders
    yarp::os::Property prop_encoders;
    prop_encoders.put("device", "remote_controlboard");
    prop_encoders.put("remote", "/icubSim/right_arm");
    prop_encoders.put("local", "/upf-localizer/encoders/right_arm");
    ok_drv = drv_right_arm.open(prop_encoders);
    if (!ok_drv)
    {
	yError() << "LocalizerModule::configure error:"
		 << "unable to open the Remote Control Board driver for the right arm";
	return false;
    }

    prop_encoders.put("remote", "/icubSim/left_arm");
    prop_encoders.put("local", "/upf-localizer/encoders/left_arm");
    ok_drv = drv_left_arm.open(prop_encoders);
    if (!ok_drv)
    {
	yError() << "LocalizerModule::configure error:"
		 << "unable to open the Remote Control Board driver for the left arm";
	return false;
    }

    prop_encoders.put("remote", "/icubSim/torso");
    prop_encoders.put("local", "/upf-localizer/encoders/torso");
    ok_drv = drv_torso.open(prop_encoders);
    if (!ok_drv)
    {
	yError() << "LocalizerModule::configure error:"
		 << "unable to open the Remote Control Board driver for the torso";
	return false;
    }

    // try to retrieve the views
    bool ok_view = drv_right_arm.view(ienc_right_arm);
    if (!ok_view || ienc_right_arm == 0)
    {
	yError() << "LocalizerModule:configure error:"
		 << "unable to retrieve the Encoders view for the right arm";
	return false;
    }
    ok_view = drv_left_arm.view(ienc_left_arm);
    if (!ok_view || ienc_left_arm == 0)
    {
	yError() << "LocalizerModule:configure error:"
		 << "unable to retrieve the Encoders view for the left arm";
	return false;
    }
    ok_view = drv_torso.view(ienc_torso);
    if (!ok_view || ienc_torso == 0)
    {
	yError() << "LocalizerModule:configure error:"
		 << "unable to retrieve the Encoders view for the torso";
	return false;
    }

    // configure forward kinematics
    right_arm_kin = iCub::iKin::iCubArm("right");
    left_arm_kin = iCub::iKin::iCubArm("left");
    right_middle = iCub::iKin::iCubFinger("right_middle");
    left_middle = iCub::iKin::iCubFinger("left_middle");

    // Limits update is not required to evaluate the forward kinematics
    // using angles from the encoders
    right_arm_kin.setAllConstraints(false);
    left_arm_kin.setAllConstraints(false);
    // Torso can be moved in general so its links have to be released
    right_arm_kin.releaseLink(0);
    right_arm_kin.releaseLink(1);
    right_arm_kin.releaseLink(2);
    left_arm_kin.releaseLink(0);
    left_arm_kin.releaseLink(1);
    left_arm_kin.releaseLink(2);

    // configure and init the UPF
    if(!upf.configure(rf))
    	return false;
    upf.init();

    // reset storage
    storage_on = false;
    resetStorage();

    // start rpc server
    rpc_port.open(rpc_port_name);
    attach(rpc_port);

    // reset flags
    estimate_available = false;
    filtering_enabled = false;
    is_first_step = true;

    return true;
}

double LocalizerModule::getPeriod()
{
    // TODO: take from the config
    return 0.01;
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

    // process the command
    if (data != YARP_NULLPTR)
	processCommand(*data);

    //
    if (data != YARP_NULLPTR)
	performFiltering(*data);

    // publish the last estimate available
    if (estimate_available)
	publishEstimate();

    return true;
}

bool LocalizerModule::close()
{
    // close ports
    port_in.close();
    port_pc.close();

    // close the driver
    drv_transform_client.close();
}

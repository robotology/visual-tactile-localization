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

// std
#include <fstream>
#include <string>

// yarp
#include <yarp/os/all.h>

// CGAL
#include <CGAL/IO/Polyhedron_iostream.h>

//
#include "headers/localizer.h"
#include "headers/unscentedParticleFilter.h"
#include "headers/geometryCGAL.h"

bool Localizer::loadMeasurements(const yarp::os::ResourceFinder &rf)
{
    // read measurements filename from the resource finder
    std::string measFileName = rf.find("measurementsFile").asString();
    if (rf.find("measurementsFile").isNull())
    {
        yError() << "measurements file not provided. Check your configuration.";
        return false;
    }

    // open the file
    std::ifstream measFile(measFileName.c_str());
    if (!measFile.is_open())
    {
	measFile.close();

        yError() << "problem reading measurements file. Check your measurements file.";
        return false;
    }

    // process the measurements file
    int state = 0;
    int n_points = 0;
    char line[255];
        
    while (!measFile.eof())
    {
	// get a new line
        measFile.getline(line,sizeof(line),'\n');
        yarp::os::Bottle b(line);
	
	// get the first item of the line
        yarp::os::Value first_item=b.get(0);

	// check if the first item is a number
        bool is_number = first_item.isInt() || first_item.isDouble();
        
        if (state == 0)
        {
	    // state 0 expect a "OFF" header
	    
	    // get header
	    std::string header;
	    if (first_item.isString())
		header = first_item.asString();		
	    else
	    {
		measFile.close();

		yError() << "invalid measurements file header. Check your measurements file.";
		return false;
	    }
	    
	    // capitalize the header
            std::transform(header.begin(),header.end(),header.begin(),::toupper);
	    
	    // check the header
            if (header == "OFF")
                state++;
	}
	else if (state == 1)
	{
	    // state 1 expect the number of measurements
	    
	    if (is_number)
	    {
		n_points = first_item.asInt();
		state++;
	    }
	    else
	    {
		measFile.close();

		yError() << "invalid number of measurements. Check your measurements file.";
		return false;
	    }
	    
	}
	else if (state == 2)
	{
	    // state 2 expect a point with at least three double coordinates
	    
	    if (b.size() >= 3)
	    {
		if (b.get(0).isDouble() &&
		    b.get(1).isDouble() &&
		    b.get(2).isDouble())
		{
		    measurements.push_back(Point(b.get(0).asDouble(),
						 b.get(1).asDouble(),
						 b.get(2).asDouble()));
		    n_points--;
		    if (n_points <= 0)
			return true;
		}
		else
		{
		    measFile.close();

		    yError() << "point with invalid coordinate found. Check your measurements file.";
		    return false;
		}

	    }
	    else
	    {
		measFile.close();

		yError() << "point with less than three coordinates found. Check your measurements file.";
		return false;
	    }
	}
    }
    return false;
}

bool Localizer::loadParameters(const yarp::os::ResourceFinder &rf)
{
    // load the number of trials
    yarp::os::Value num_trials_value = rf.find("numTrials");
    if (rf.find("numTrials").isNull())
    {
        yError() << "number of trials not provided.";
        return false;
    }
    if (!num_trials_value.isInt())
    {
        yError() << "invalid number of trials provided.";
        return false;
    }
    num_trials = num_trials_value.asInt();
    yInfo() << "Localizer: number of particles:" << num_trials;    

    // load the fixed number of contact points per time step
    yarp::os::Value n_contacts_value = rf.find("numContacts");
    if (rf.find("numContacts").isNull())
    {
        yError() << "number of contact points not provided.";
        return false;
    }
    if (!n_contacts_value.isInt())
    {
        yError() << "invalid number of contact points provided.";
        return false;
    }
    n_contacts = n_contacts_value.asInt();
    yInfo() << "Localizer: number of contact points per step:"
	    << n_contacts;

    // load the real pose
    real_pose.resize(6, 0.0);    
    yarp::os::Value real_pose_value = rf.find("realPose");
    if (!rf.find("realPose").isNull())
    {	
	yarp::os::Bottle *b = real_pose_value.asList();
	if (b->size() >= 6)
	{
	    for(size_t i; i<6; i++)
	    {
		if(!b->get(i).isDouble())
		{
		    yError() << "invalid real pose provided.";
		    return false;
		}
		real_pose[i] = b->get(i).asDouble();
	    }
	}
	else
	{
	    yError() << "real pose should contain 6 scalars.";
	    return false;
	}
    }
    yInfo() << "Localizer: real pose"
	    << real_pose.toString();

    return true;
}

bool Localizer::saveModel(const Polyhedron &model,
			  const std::string &file_name)
{    
    // get the output path if specified
    std::string outputPath;    
    yarp::os::Value path_value = rf->find("modelOutputPath");
    if (rf->find("modelOutputPath").isNull())
	outputPath = "../../outputs/";
    else
	outputPath = path_value.asString();

    // file name
    std::string outputFileName = outputPath + file_name;

    // save the model
    std::ofstream fout(outputFileName.c_str());
    if(fout.is_open())
    	fout << model;
    else
    {
	fout.close();
	
    	yError() << "problem opening transformed model output file!";
	return false;
    }
    
    fout.close();

    return true;
}

bool Localizer::saveResults(const std::vector<Results> &results)
{
    // get the output path if specified
    std::string outputPath;    
    yarp::os::Value path_value = rf->find("trialsOutputPath");
    if (rf->find("trialsOutputPath").isNull())
	outputPath = "../../outputs/";
    else
	outputPath = path_value.asString();

    // file name
    std::string outputFileName = outputPath + "results.csv";

    // save the results
    std::ofstream fout(outputFileName.c_str());
    if(fout.is_open())
    {
	// print the CSV header
	fout << "trial;"
	     << "perf_index;"
	     << "time_no_estimate;"
	     << "time;"
	     << "sol_x;"   << "sol_y;"     << "sol_z;"
	     << "sol_phi;" << "sol_theta;" << "sol_psi"
	     << std::endl;

	// save data for each trial
	for(size_t i=0; i<results.size(); i++)
	{
	    fout << i <<";"
		 << results[i].perf_index << ";"
		 << results[i].tf_no_map << ";"
		 << results[i].tf << ";"		
		 << results[i].estimate[0] << ";"
		 << results[i].estimate[1] << ";"
		 << results[i].estimate[2] << ";"
		 << results[i].estimate[3] << ";"
		 << results[i].estimate[4] << ";"
		 << results[i].estimate[5] << ";"
		 << std::endl;
	}
    }
    else
    {
	fout.close();
	
    	yError() << "problem opening transformed model output file!";
	return false;
    }
    
    fout.close();

    return true;
    
}

bool Localizer::configure(yarp::os::ResourceFinder &rf)
{
    // save reference to resource finder
    this->rf = &rf;
    
    // load the parameters
    if(!loadParameters(rf))
    	return false;
    
    // load the measurements
    std::vector<Point> meas;
    if(!loadMeasurements(rf))
    	return false;

    // allocate storage for results
    results.assign(num_trials, Results());

    // instantiate the UPF
    upf = new UnscentedParticleFilter;

    // configure the UPF
    if(!upf->configure(rf))
	return false;
    upf->setRealPose(real_pose);

    return true;
}

bool Localizer::updateModule()
{
    // execute all the trials
    for(size_t i=0; i<num_trials; i++)
    {
	yInfo() << "Localizer: performing trial no." << i;
	
	// save initial time
	double t0 = yarp::os::Time::now();
	
	// init the filter
	upf->init();

    	// process all contact points
    	for(size_t k=0; k+n_contacts-1< measurements.size(); k+=n_contacts)	
    	{
	    // check if the module was stopped
	    if(isStopping())
		return false;
	    
    	    // simulate n_contacts contact points per time step
    	    Measure m;
    	    for(size_t j=0; j<n_contacts; j++)
    		m.push_back(measurements[k+j]);

    	    // set new measure
    	    upf->setNewMeasure(m);

    	    // step
    	    upf->step();
    	}

	// save partial time
	double tf_no_estimate = yarp::os::Time::now() - t0;

	// extract MAP estimate
	yarp::sig::Vector estimate = upf->getEstimate();

	// save final_time
	double tf = yarp::os::Time::now() - t0;

	// eval performance index
	double perf_index = upf->evalPerformanceIndex(estimate,
						      measurements);
	// save results
	results[i].estimate = estimate;
	results[i].perf_index = perf_index;
	results[i].tf_no_map = tf_no_estimate;
	results[i].tf = tf;

	// save the transformed model
	Polyhedron transformed;
	upf->transformObject(estimate, transformed);
	saveModel(transformed, "transformedModelTrial"
		               + std::to_string(i)
		               + ".off");
    }

    // save the results of all the trials
    saveResults(results);

    yInfo() << "All trials executed";

    // return false so that the module closes
    return false;
}

bool Localizer::close()
{
    // delete the UPF instance
    delete upf;
}

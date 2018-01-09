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
#include <string>

// yarp
#include <yarp/os/all.h>

//
#include "headers/localizer-motion.h"

bool loadFileName(const std::string &tag, const yarp::os::ResourceFinder &rf,
		  std::string &file_name)
{
    // load file name from the resource finder
    file_name = rf.find(tag).asString();
    if (rf.find(tag).isNull())
    {
        yError() << "file name for tag" << tag << "not provided. Check your configuration.";
        return false;
    }
    
    return true;
}

/*
 * main
 */ 
int main(int argc, char **argv)
{
    // instantiate the resource finder
    yarp::os::ResourceFinder rf;
    rf.configure(argc,argv);

    // instantiate the localizer
    LocalizerMotion localizer;

    // load several file names
    std::string model_file_name;
    std::string contact_points_1_file_name;
    std::string contact_points_2_file_name;
    loadFileName("modelFile", rf, model_file_name);
    loadFileName("auxCloud1File", rf, contact_points_1_file_name);
    loadFileName("auxCloud2File", rf, contact_points_2_file_name);    

    // instantiate fake point cloud engines
    FakePointCloud pc_whole;
    FakePointCloud pc_contacts_1;
    FakePointCloud pc_contacts_2;
    pc_whole.loadObjectModel(model_file_name);
    pc_contacts_1.loadObjectModel(contact_points_1_file_name);
    pc_contacts_2.loadObjectModel(contact_points_2_file_name);

    // instantiate static pose and motion generator
    StaticMotionGenerator static_mg;
    PolynomialMotionGenerator motion_mg;

    /*
     * setup the localization experiment
     */
    double step_time = 0.01;

    // static phase 1
    LocalizationPhase static1(LocalizationType::Static,
			      6, 1, &static_mg, &pc_whole);
    // set displacement from ref point to center
    static1.displ[0] = 0.0283;
    static1.displ[1] = 0.0083;
    static1.displ[2] = -0.0029;
    // set initial conditions
    static1.ref_pos_0[0] = 0.1;
    static1.ref_pos_0[1] = 0.1;
    static1.yaw_0 = 0.0;
    // set number of points in point cloud
    static1.num_points = 150;

    // motion phase 1
    LocalizationPhase motion1(LocalizationType::Motion,
			      2.5, step_time, &motion_mg,
			      &pc_contacts_1,true);
    // set position and angular displacement
    motion1.delta_pos[0] = 0.2;
    motion1.delta_yaw = -M_PI/8.0;

    // static phase 2
    LocalizationPhase static2(LocalizationType::Static,
			      6, 1, &static_mg, &pc_whole,
			      true);
    static2.num_points = 150;

    // motion phase 2
    LocalizationPhase motion2(LocalizationType::Motion,
			      2.5, step_time, &motion_mg,
			      &pc_contacts_2);
    // change reference point
    motion2.displ[0] = -0.0283;
    motion2.displ[1] = 0.0083;
    motion2.displ[2] = -0.0029;
    // set position and angular displacement	
    motion2.delta_pos[1] = 0.2;
    motion2.delta_yaw = - M_PI/8.0;	
	
    // static phase 3
    LocalizationPhase static3(LocalizationType::Static,
			      6, 1, &static_mg, &pc_whole,
			      true);
    // set number of points in point cloud
    static3.num_points = 150;

    /*
     *
     */

    // add phases to the localization experiment
    localizer.addLocPhase(static1);
    localizer.addLocPhase(motion1);
    localizer.addLocPhase(static2);
    localizer.addLocPhase(motion2);
    localizer.addLocPhase(static3);

    // set the step time
    localizer.setStepTime(step_time);

    // run the localizer
    localizer.runModule(rf);
}
    

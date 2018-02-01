/******************************************************************************
 *                                                                            *
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga <nicolapiga@gmail.com>
 */

// std
#include <string>

// yarp
#include <yarp/os/all.h>

//
#include "headers/localizer-module.h"
#include "headers/filterData.h"

/*
 * main
 */ 
int main(int argc, char **argv)
{
    // instantiate the resource finder
    yarp::os::ResourceFinder rf;
    rf.configure(argc,argv);

    // instantiate the localizer
    LocalizerModule localizer;

    // run the localizer
    localizer.runModule(rf);

}
    

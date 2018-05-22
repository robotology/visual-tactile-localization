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

/*
 * main
 */
int main(int argc, char **argv)
{
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError() << "LocalizerModule: cannot find YARP!";
        return 1;
    }

    // instantiate the resource finder
    yarp::os::ResourceFinder rf;
    rf.setDefaultConfigFile("localizer_config.ini");
    rf.configure(argc,argv);

    // instantiate the localizer
    LocalizerModule localizer;

    // run the localizer
    localizer.runModule(rf);

}


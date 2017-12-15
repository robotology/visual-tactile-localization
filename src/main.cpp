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

// yarp
#include <yarp/os/all.h>

//
#include "headers/localizer-motion.h"

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

    localizer.runModule(rf);
}
    

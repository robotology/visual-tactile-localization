/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <Viewer.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include <string>

using namespace yarp::os;

int main(int argc, char** argv)
{
    const std::string log_ID = "[MAIN]";
    const std::string port_prefix = "object-tracking-viewer";
    
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError() << log_ID << "Yarp is not available.";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("config.ini");
    // rf.setDefaultContext("object-tracking-viewer");
    rf.configure(argc, argv);

    Viewer(port_prefix, rf);

    return EXIT_SUCCESS;
}



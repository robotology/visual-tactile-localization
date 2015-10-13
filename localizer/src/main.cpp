
/*
 * Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Giulia Vezzani
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#include <iostream>
#include <fstream>
#include <string>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/math/Rand.h>
#include <iCub/ctrl/math.h>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/skinDynLib/skinContactList.h>
#include <CGAL/IO/Polyhedron_iostream.h>


#include "headers/localizer.h"
#include "headers/scalingSeries.h"
#include "headers/unscentedParticleFilter.h"



YARP_DECLARE_DEVICES(icubmod)
using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;
using namespace iCub::iKin;
using namespace iCub::skinDynLib;

int main(int argc, char *argv[])
{
    
    ResourceFinder rf;
    rf.configure(argc,argv);
    rf.setDefaultContext("../../");
   
         

    
    Localizer *loc5=new UnscentedParticleFilter();
    loc5->configure(rf);
    yarp::sig::Vector result2=loc5->localization();
    loc5->saveData(result2);
   
    
    delete loc5;
    
    
    Localizer *loc4=new ScalingSeries();
    loc4->configure(rf);
    yarp::sig::Vector result=loc4->localization();
    loc4->saveData(result);
    
    delete loc4;
    
 
    
    
    
    return 0;
}

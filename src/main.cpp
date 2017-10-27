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
    int numTrials;

    if(argc>1)
	numTrials=atoi(argv[1]); 
    else
	numTrials=1;

    yarp::sig::Matrix solutions;
    yarp::sig::Vector error_indices;

    if(!strcmp(argv[2],"mupf"))
    {
        solutions.resize(numTrials,4);
        for(size_t i=0; i<numTrials; i++)
        {
            Localizer *loc5=new UnscentedParticleFilter();
            loc5->configure(rf);
            error_indices=loc5->localization();
            loc5->saveData(error_indices,i);
            solutions(i,0)=error_indices[6];

            delete loc5;
        }

        Localizer *loc5=new UnscentedParticleFilter();
        loc5->saveStatisticsData(solutions);

        delete loc5;
    }
    else
    {
        error_indices.resize(0.0,8);
        solutions.resize(numTrials,2);
        for(size_t i=0; i<numTrials-1; i++)
        {
	    Localizer *loc4=new ScalingSeries();
	    loc4->configure(rf);
	    error_indices=loc4->localization();
	    loc4->saveData(error_indices,i);
	    solutions(i,0)=error_indices[6];
	    solutions(i,1)=error_indices[7];
	    delete loc4;
	}
	
	Localizer *loc4=new ScalingSeries();
	loc4->configure(rf);
	error_indices=loc4->localization();
	loc4->saveData(error_indices,numTrials-1);
	solutions(numTrials-1,0)=error_indices[6];
	solutions(numTrials-1,1)=error_indices[7];
	loc4->saveStatisticsData(solutions);
	delete loc4;

    }
    return 0;
}

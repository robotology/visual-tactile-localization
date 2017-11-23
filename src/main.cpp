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

    solutions.resize(numTrials,8);

    if(strcmp(argv[2],"standard_mupf") &&
       strcmp(argv[2],"experimental_mupf"))
       return 0;
       
    for(size_t i=0; i<numTrials; i++)
    {
	UnscentedParticleFilter *upf=new UnscentedParticleFilter();

	// initialize
	upf->configure(rf);
	upf->init();

	// solve
	if(!strcmp(argv[2],"standard_mupf"))
	{
	    upf->solve_standard_mupf();
	}
	else if(!strcmp(argv[2],"experimental_mupf"))
	{
	    upf->solve_experimental_mupf();
	}

	error_indices=upf->finalize();

	// save data
	upf->saveData(error_indices,i);
	// save error index
	solutions(i,0)=error_indices[6];
	// save execution time
	// (including MAP extraction)
	solutions(i,1)=error_indices[7];
	// save solutions
	solutions(i,2)=error_indices[0];
	solutions(i,3)=error_indices[1];
	solutions(i,4)=error_indices[2];
	solutions(i,5)=error_indices[3];	
	solutions(i,6)=error_indices[4];
	solutions(i,7)=error_indices[5];
		
	delete upf;
    }

    Localizer *upf=new UnscentedParticleFilter();
    upf->configure(rf);
    
    upf->saveTrialsData(solutions);
        
    delete upf;

    return 0;
}

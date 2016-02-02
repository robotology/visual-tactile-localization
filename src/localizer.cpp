/*
 * Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Giulia Vezzani
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <cmath>

#include <yarp/os/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <yarp/os/ResourceFinder.h>
#include <iCub/ctrl/math.h>

#include "headers/localizer.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;

/***********************************************************************************/
bool Localizer::readCenter(const string &tag, Vector &center0)
    {
      
        if (Bottle *b=this->rf->find(tag.c_str()).asList())
        {
            if (b->size()>=3)
            {   
              
                center0[0]=b->get(0).asDouble();
                center0[1]=b->get(1).asDouble();
				center0[2]=b->get(2).asDouble();
                return true;
            }
        }
        
       
        return false;
    }
    
/***********************************************************************************/        
bool Localizer::readRadius(const string &tag, Vector &radius0)
{
    if (Bottle *b=this->rf->find(tag.c_str()).asList())
    {
       
        if (b->size()>=3)
        {
            
            radius0[0]=b->get(0).asDouble();
           
            radius0[1]=b->get(1).asDouble();
            
			radius0[2]=b->get(2).asDouble();
			
				
            return true;
        }
    }
        
    return false;
}
    
/***********************************************************************************/      
bool Localizer::readDiagonalMatrix(const string &tag, Vector &diag, const int &dimension)
{
    if (Bottle *b=this->rf->find(tag.c_str()).asList())
    {
        if (b->size()>=dimension)
        {
            for(size_t i; i<dimension; i++)
                diag[i]=b->get(i).asDouble();
             
				
            return true;
        }
    }
        
    return false;
}
    
/***********************************************************************************/        
void Localizer::sendData( const yarp::sig::Vector &ms_particle)
{
    BufferedPort<Bottle> dataOutPort;
      
    string outPortName=this->rf->find("outPortName").asString().c_str();
    
    if(this->rf->find("outPortName").isNull())
        outPortName=this->rf->check("outPortName",Value("output_port")).asString().c_str();
            
    dataOutPort.open(("/"+outPortName+"/data:o").c_str());
       
    Bottle &dataOut=dataOutPort.prepare();
	dataOut.clear();
	   
	for(size_t i=0; i<6; i++)
   {
		 dataOut.addDouble(ms_particle(i));
	}
		    
	dataOutPort.write();
    dataOutPort.close();
        
};

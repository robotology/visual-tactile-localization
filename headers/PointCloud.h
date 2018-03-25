/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga <nicolapiga@gmail.com>
 */

#ifndef POINTCLOUD_H

#include <yarp/sig/Vector.h>

#define NOT_IMPLEMENTED_TAG -1

struct PointCloudItem
{
    double x;
    double y;
    double z;
};

struct RGBPointCloudItem
{
    double x;
    double y;
    double z;
    unsigned char r;
    unsigned char g;
    unsigned char b;
};

/*
 * yarp::sig::VectorOf<T>::getBottleTag calls the function
 * template<class T> BottleTagMap that needs to be instantiated
 * for each type T used in VectorOf<T> to ensure bottle compatible
 * serialization. Since this feature is of no interest the method
 * is overriden as follows. Drawback is that 'yarp read' cannot
 * be used.
 */
class PointCloud : public yarp::sig::VectorOf<PointCloudItem>
{
    int getBottleTag() const YARP_OVERRIDE
    {
	return NOT_IMPLEMENTED_TAG;
    }
};

class RGBPointCloud : public yarp::sig::VectorOf<RGBPointCloudItem>
{
    int getBottleTag() const YARP_OVERRIDE
    {
	return NOT_IMPLEMENTED_TAG;	
    }
};

#endif

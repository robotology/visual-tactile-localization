/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef __PERCEPTIVEMODELS_PORTS_H__
#define __PERCEPTIVEMODELS_PORTS_H__

#include <yarp/os/Mutex.h>
#include <yarp/os/Value.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>


namespace iCub
{

namespace perception
{

class Port : public yarp::os::BufferedPort<yarp::os::Bottle>
{
protected:
    yarp::os::Mutex  mutex;
    yarp::os::Bottle bottle;

    void onRead(yarp::os::Bottle &bottle);

public:
    Port();
    yarp::os::Value getValue(const int index);
};


}

}

#endif



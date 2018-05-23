/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga <nicolapiga@gmail.com>
 */

#ifndef FWD_KIN_EXT_H
#define FWD_KIN_EXT_H

// yarp
#include <yarp/sig/Vector.h>
#include <yarp/dev/ControlBoardInterfaces.h>

// icub-main
#include <iCub/iKin/iKinFwd.h>

// std
#include <string>
#include <deque>

namespace iCub
{
    namespace iKin
    {
	class iCubFingerExt;
    }
}

class iCub::iKin::iCubFingerExt : public iCub::iKin::iCubFinger
{
public:
    iCubFingerExt();
    iCubFingerExt(const std::string &  _type);
    void allocate(const std::string &  _type) override;
    bool getChainJoints(const yarp::sig::Vector &motorEncoders,
			yarp::sig::Vector &chainJoints) override;
    bool getChainJoints(const yarp::sig::Vector &motorEncoders,
			const yarp::sig::Vector &jointEncoders,
			yarp::sig::Vector &chainJoints,
			const yarp::sig::Matrix &jointEncodersBounds = yarp::math::zeros(1, 2)) override;
    bool alignJointsBounds(const std::deque<yarp::dev::IControlLimits*> &lim) override;
};

#endif

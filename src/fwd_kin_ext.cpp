/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga <nicolapiga@gmail.com>
 */

// icub-main
#include <iCub/iKin/iKinFwd.h>

//
#include <cmath>

//
#include "headers/fwd_kin_ext.h"

using namespace iCub::ctrl;

iCub::iKin::iCubFingerExt::iCubFingerExt() : iCub::iKin::iCubFinger() {};

iCub::iKin::iCubFingerExt::iCubFingerExt(const std::string &  _type) : iCub::iKin::iCubFinger(_type)
{
    allocate(_type);
};

void iCub::iKin::iCubFingerExt::allocate (const std::string &  _type)
{
    // after call to iCubFinger::allocate in the base ctor
    // the finger variable contains the name of the finger
    if (finger=="ring" || finger=="little")
    {
        // in case the super class already implements ring and little fingers
        // it is required to clean the chain
        this->clear();
        //

	if (finger=="ring")
	{
	    H0(0,0)=0.9998; H0(0,1)=0.0192;  H0(0,2)=0.0;     H0(0,3)=0.001569230;
	    H0(1,0)=0.0191; H0(1,1)=-0.9960; H0(1,2)=0.0872;  H0(1,3)=0.007158757;
	    H0(2,0)=0.0017; H0(2,1)=-0.0871; H0(2,2)=-0.9962; H0(2,3)=-0.011458935;
	    H0(3,0)=0.0;    H0(3,1)=0.0;     H0(3,2)=0.0;     H0(3,3)=1.0;

	    if (hand=="left")
	    {
		H0(2,0)=-H0(2,0);
		H0(2,1)=-H0(2,1);
		H0(1,2)=-H0(1,2);
		H0(2,3)=-H0(2,3);

		pushLink(new iKinLink(0.0148, 0.0, M_PI/2.0, -20.0*CTRL_DEG2RAD, 0.0, 20.0*CTRL_DEG2RAD));
	    }
	    else
		pushLink(new iKinLink(0.0148, 0.0, -M_PI/2.0, -20.0*CTRL_DEG2RAD, 0.0, 20.0*CTRL_DEG2RAD));

	    pushLink(new iKinLink(0.0259, 0.0,       0.0,                0.0, 0.0, 90.0*CTRL_DEG2RAD));
	    pushLink(new iKinLink(0.0220, 0.0,       0.0,                0.0, 0.0, 90.0*CTRL_DEG2RAD));
	    pushLink(new iKinLink(0.0168, 0.0, -M_PI/2.0,                0.0, 0.0, 90.0*CTRL_DEG2RAD));
	}
	else if (finger == "little")
	{
	    H0(0,0)=0.9998; H0(0,1)=0.0192;  H0(0,2)=0.0;     H0(0,3)=-0.00042147;
	    H0(1,0)=0.0191; H0(1,1)=-0.9960; H0(1,2)=0.0872;  H0(1,3)=0.0232755;
	    H0(2,0)=0.0017; H0(2,1)=-0.0871; H0(2,2)=-0.9962; H0(2,3)=-0.00956329;
	    H0(3,0)=0.0;    H0(3,1)=0.0;     H0(3,2)=0.0;     H0(3,3)=1.0;

	    if (hand=="left")
	    {
		H0(2,0)=-H0(2,0);
		H0(2,1)=-H0(2,1);
		H0(1,2)=-H0(1,2);
		H0(2,3)=-H0(2,3);

		pushLink(new iKinLink(0.0148, 0.0, M_PI/2.0, -20.0*CTRL_DEG2RAD, 0.0, 20.0*CTRL_DEG2RAD));
	    }
	    else
		pushLink(new iKinLink(0.0148, 0.0, -M_PI/2.0, -20.0*CTRL_DEG2RAD, 0.0, 20.0*CTRL_DEG2RAD));

	    pushLink(new iKinLink(0.0219, 0.0,       0.0,                0.0, 0.0, 90.0*CTRL_DEG2RAD));
	    pushLink(new iKinLink(0.0190, 0.0,       0.0,                0.0, 0.0, 90.0*CTRL_DEG2RAD));
	    pushLink(new iKinLink(0.0168, 0.0, -M_PI/2.0,                0.0, 0.0, 90.0*CTRL_DEG2RAD));
	}
	setH0(H0);
    }
}

bool iCub::iKin::iCubFingerExt::getChainJoints(const yarp::sig::Vector &motorEncoders,
					       yarp::sig::Vector &chainJoints)
{
    if (finger != "ring" && finger != "little")
	// getChainJoints would return false in case of ring/little fingers
	return iCub::iKin::iCubFinger::getChainJoints(motorEncoders, chainJoints);

    if ((motorEncoders.length()!=9) && (motorEncoders.length()!=16))
        return false;

    int offs=(motorEncoders.length()==16?7:0);

    if (finger == "ring" || finger == "little")
    {
	chainJoints.resize(4);
	// adduction/abduction
	chainJoints[0]=motorEncoders[offs+0]/3.0;
	// 1 DoF spanning six joints on two fingers
	chainJoints[1]=motorEncoders[offs+8]/3.0;
	chainJoints[3]=chainJoints[2]=chainJoints[1];
    }

    return true;
}

bool iCub::iKin::iCubFingerExt::getChainJoints(const yarp::sig::Vector &motorEncoders,
					       const yarp::sig::Vector &jointEncoders,
					       yarp::sig::Vector &chainJoints,
					       const yarp::sig::Matrix &jointEncodersBounds)
{
    if (finger != "ring" && finger != "little")
	// getChainJoints would return false in case of ring/little fingers
	return iCub::iKin::iCubFinger::getChainJoints(motorEncoders, jointEncoders,
						      chainJoints, jointEncodersBounds);

    if (((motorEncoders.length()!=9) && (motorEncoders.length()!=16)) ||
        (jointEncoders.length()<15) || (jointEncodersBounds.cols()<2))
        return false;

    int offs=(motorEncoders.length()==16?7:0);

    yarp::sig::Matrix bounds=jointEncodersBounds;
    if (bounds.rows()!=jointEncoders.length())
    {
        bounds=yarp::math::zeros(jointEncoders.length(),2);
        for (size_t r=0; r<jointEncoders.length(); r++)
            bounds(r,0)=255.0;
    }

    if (finger == "ring")
    {
	chainJoints.resize(4);
	// adduction/abduction
	chainJoints[0]=motorEncoders[offs+0]/3.0;

	// proximal and distal joints
        for (size_t i=1; i<chainJoints.length(); i++)
        {
            double c=0.0;
            double span=bounds(i+8,1)-bounds(i+8,0);
            if (span>0.0)
                c=std::min(1.0,std::max(0.0,(jointEncoders[i+8]-bounds(i+8,0))/span));
            else if (span<0.0)
                c=1.0-std::min(1.0,std::max(0.0,(bounds(i+8,1)-jointEncoders[i+8])/span));
            chainJoints[i]=CTRL_RAD2DEG*(c*((*this)[i].getMax()-(*this)[i].getMin())+(*this)[i].getMin());
        }
    }
    else if (finger == "little")
    {
	chainJoints.resize(4);
	// adduction/abduction
	chainJoints[0]=motorEncoders[offs+0]/3.0;

	// proximal and distal joints
        for (size_t i=1; i<chainJoints.length(); i++)
        {
            double c=0.0;
            double span=bounds(i+11,1)-bounds(i+11,0);
            if (span>0.0)
                c=std::min(1.0,std::max(0.0,(jointEncoders[i+11]-bounds(i+11,0))/span));
            else if (span<0.0)
                c=1.0-std::min(1.0,std::max(0.0,(bounds(i+11,1)-jointEncoders[i+11])/span));
            chainJoints[i]=CTRL_RAD2DEG*(c*((*this)[i].getMax()-(*this)[i].getMin())+(*this)[i].getMin());
        }
    }
    else
	return false;

    return true;
}

bool iCub::iKin::iCubFingerExt::alignJointsBounds(const std::deque<yarp::dev::IControlLimits*> &lim)
{
    if (finger != "ring" && finger != "little")
	return iCub::iKin::iCubFinger::alignJointsBounds(lim);

    if (lim.size() < 1)
        return false;

    yarp::dev::IControlLimits &limFinger=*lim[0];
    double min, max;

    if (finger=="ring" || finger == "little")
    {
	// adduction/abduction
        if (!limFinger.getLimits(7,&min,&max))
            return false;

        (*this)[0].setMin(CTRL_DEG2RAD*min);
	// adduction/abduction is common to index, ring and little
        (*this)[0].setMax(CTRL_DEG2RAD*max/3.0);

	// ring and little finger flexion
        if (!limFinger.getLimits(15,&min,&max))
            return false;

	// 1 DoF spanning six joints on two fingers
        (*this)[1].setMin(CTRL_DEG2RAD*min);
        (*this)[1].setMax(CTRL_DEG2RAD*max/3.0);
        (*this)[2].setMin(CTRL_DEG2RAD*min);
        (*this)[2].setMax(CTRL_DEG2RAD*max/3.0);
        (*this)[3].setMin(CTRL_DEG2RAD*min);
        (*this)[3].setMax(CTRL_DEG2RAD*max/3.0);
    }
	return false;

    return true;
}

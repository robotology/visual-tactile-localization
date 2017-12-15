/******************************************************************************
 *                                                                            *
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga <nicolapiga@gmail.com>
 */

// yarp
#include <yarp/math/Math.h>

//
#include "headers/motionGenerator.h"

using namespace yarp::math;

void MotionGenerator::setPeriod(const double &time)
{
    dt = time;
}

void MotionGenerator::setDuration(const double &time)
{
    T = time;
}

void MotionGenerator::setInitialRefPosition(const yarp::sig::Vector &pos)
{
    ref_pos_0 = pos;
}

void MotionGenerator::setInitialYaw(const double &yaw)
{
    yaw_0 = yaw;
}

void MotionGenerator::setDisplToCenter(const yarp::sig::Vector &displ)
{
    displToCenter = displ;
}

bool MotionGenerator::step()
{
    // step time
    t += dt;

    // check if the motion is ended
    if (t >= T)
	return true;
    else
	return false;
}

void MotionGenerator::reset()
{
    // set current time to -dt so that
    // the motion starts at t=0 when step() is called
    t = -dt;
}

void StaticMotionGenerator::getMotion(yarp::sig::Vector &pos,
				      yarp::sig::Vector &vel,
				      yarp::sig::Vector &ref_vel,				      
				      double &yaw)
{
    // compute initial yaw rotation
    yarp::sig::Matrix yaw_rot;
    yarp::sig::Vector axis_angle(4, 0.0);

    axis_angle[2] = 1.0;
    axis_angle[3] = yaw_0;
	
    yaw_rot = yarp::math::axis2dcm(axis_angle).submatrix(0, 2,
							 0, 2);

    // return the initial position of the center of the object    
    pos = ref_pos_0 + yaw_rot * displToCenter;

    // return the initial yaw angle of the object
    yaw = yaw_0;

    // return zero velocities
    ref_vel = vel = 0;
}

void PolynomialMotionGenerator::setFinalRefPosition(const yarp::sig::Vector &pos)
{
    ref_pos_f = pos;
}

void PolynomialMotionGenerator::setFinalYaw(const double &yaw)
{
    yaw_f = yaw;
}

void PolynomialMotionGenerator::initTrajectory()
{
    // set initial values
    a0.setSubvector(0, ref_pos_0);
    a0[3] = yaw_0;

    // set final values
    a3.setSubvector(0, ref_pos_f);
    a3[3] = yaw_f;

    // eval the difference between
    // initials and finals
    a5 = a4 = a3 = a0 - a3;

    // eval the polynomial trajectory constants
    a3 *= (-10.0 / pow(T, 3));
    a4 *= (15.0 / pow(T, 4));
    a5 *= (-6.0 / pow(T, 5));

    // set the initial value of posCenter
    yarp::sig::Matrix yaw_rot;
    yarp::sig::Vector axis_angle(4, 0.0);

    axis_angle[2] = 1.0;
    axis_angle[3] = yaw_0;
	
    yaw_rot = yarp::math::axis2dcm(axis_angle).submatrix(0, 2,
							 0, 2);
    posCenter = ref_pos_0 + yaw_rot * displToCenter;

}

bool PolynomialMotionGenerator::step()
{
    // step the time
    bool traj_ended = MotionGenerator::step();
    
    // eval trajectory
    traj = a0
	 + a3 * pow(t, 3.0)
	 + a4 * pow(t, 4.0)
	 + a5 * pow(t, 5.0);

    traj_derivative = 3.0 * a3 * pow(t, 2.0)
	            + 4.0 * a4 * pow(t, 3.0)
    	            + 5.0 * a5 * pow(t, 4.0);

    return traj_ended;
}

void PolynomialMotionGenerator::getMotion(yarp::sig::Vector &pos,
					  yarp::sig::Vector &vel,
					  yarp::sig::Vector &ref_vel,					  
                                          double &yaw)
{
    // eval position using forward Euler integration
    pos = posCenter = posCenter + velCenter * dt;

    // get current yaw and yaw_dot
    yaw = traj[3];
    double yaw_dot = traj_derivative[3];

    // eval velocity

    // eval 2x2 yaw rotation matrix
    yarp::sig::Matrix yaw_rot;
    yarp::sig::Vector axis_angle(4, 0.0);

    axis_angle[2] = 1.0;
    axis_angle[3] = yaw;
	
    yaw_rot = yarp::math::axis2dcm(axis_angle).submatrix(0, 1,
							 0, 1);

    // velocity due to rotation
    yarp::sig::Vector vel_rot(3, 0.0);
    yarp::sig::Vector tmp(2, 0.0);
    tmp = yaw_rot * displToCenter.subVector(0,1);
    vel_rot[0] = -tmp[1] * yaw_dot;
    vel_rot[1] =  tmp[0] * yaw_dot;

    // total velocity
    ref_vel = traj_derivative.subVector(0, 2);    
    vel = velCenter = ref_vel + vel_rot;

}

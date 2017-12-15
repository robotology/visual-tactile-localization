/******************************************************************************
 *                                                                            *
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga <nicolapiga@gmail.com>
 */

#ifndef MOTION_GENERATOR_H
#define MOTION_GENERATOR_H

// yarp
#include <yarp/sig/all.h>

/*
 * Class for motion generation.
 *
 */
class MotionGenerator
{
protected:

    // sampling time
    double dt;

    // current time
    double t;

    // trajectory duration
    double T;

    // initial yaw angle
    double yaw_0;

    // initial position of reference point
    yarp::sig::Vector ref_pos_0;

    // displacement from the reference point
    // to the 'center' of the object
    // i.e. the point having coordinates (0,0,0) in
    // body reference frame
    yarp::sig::Vector displToCenter;
    
public:
    /*
     * Constructor.
     */
    MotionGenerator () : ref_pos_0(3, 0.0),
	                 displToCenter(3, 0.0),
	                 t(0.0),
	                 T(0.0),	
	                 yaw_0(0) { };
    /*
     * Set the sampling time of the generator.
     * @param dt the sampling time
     */
    void setPeriod(const double &dt);

    /*
     * Set the duration of the motion.
     * @param T the duration
     */
    void setDuration(const double &T);

    /*
     * Set the initial position of the reference point
     * @param pos yarp::sig::Vector containing the position
     */
    void setInitialRefPosition(const yarp::sig::Vector &pos);

    /*
     * Set the initial yaw angle of the object
     * @param yaw the initial angle
     */
    void setInitialYaw(const double &yaw);

    /*
     * Set the displacement from the reference point to the center
     * of the object
     * @param displ yarp::sig::Vector containing the displacement
     */
    void setDisplToCenter(const yarp::sig::Vector &displ);

    /*
     * Perform a step in the motion.
     * @return true if the motion is ended
     */
    virtual bool step();

    /*
     * Reset the current time to zero.
     */
    void reset();

    /*
     * Get the current position and velocity of the center of
     * the object and the velocity of the reference point.
     * @param pos yarp::sig::Vector where position is stored
     * @param vel yarp::sig::Vector where velocity is stored
     * @param ref_vel yarp::sig::Vector where velocity is stored
     * @param yaw yaw attitude of the object
     */
    virtual void getMotion(yarp::sig::Vector &pos,
			   yarp::sig::Vector &vel,
			   yarp::sig::Vector &ref_vel,			   
	                   double &yaw) = 0;
};

/*
 * Class for static motion generation.
 *
 */
class StaticMotionGenerator : public MotionGenerator
{
public:
    /*
     * Get the initial pose of the object and its zero velocity
     * @param pos yarp::sig::Vector where position is stored
     * @param vel yarp::sig::Vector where velocity is stored
     * @param yaw double where the yaw angle is stored
     */
    void getMotion(yarp::sig::Vector &pos,
		   yarp::sig::Vector &vel,
		   yarp::sig::Vector &ref_vel,
	           double &yaw) override;
};

/*
 * Class for polynomial motion generation.
 *
 */
class PolynomialMotionGenerator : public MotionGenerator
{
private:
    // final yaw angle
    double yaw_f;

    // final position of reference point
    yarp::sig::Vector ref_pos_f;

    // trajectory constants
    yarp::sig::Vector a0;
    yarp::sig::Vector a3;
    yarp::sig::Vector a4;
    yarp::sig::Vector a5;

    // current value of the polynomial trajectory
    // and its derivative
    yarp::sig::Vector traj;
    yarp::sig::Vector traj_derivative;

    // current position of the center of the object
    yarp::sig::Vector posCenter;

    // current velocity of the center of the object
    yarp::sig::Vector velCenter;

public:
    /*
     * Constructor.
     */
    PolynomialMotionGenerator() : ref_pos_f(3, 0.0),
	                          yaw_f(0.0),
	                          traj(4, 0.0),
	                          traj_derivative(4, 0.0),
	                          a0(4, 0.0),
	                          a3(4, 0.0),
	                          a4(4, 0.0),
	                          a5(4, 0.0),
	                          posCenter(3, 0.0),
	                          velCenter(3, 0.0) { };
    /*
     * Set the final position of the reference point
     * @param pos yarp::sig::Vector containing the position
     */
    void setFinalRefPosition(const yarp::sig::Vector &pos);

    /*
     * Set the final yaw angle of the object
     * @param yaw the final angle
     */
    void setFinalYaw(const double &yaw);

    /*
     * Initialize the trajectory constants.
     */
    void initTrajectory();

    /*
     * Perform a step in the motion.
     * The polynomial trajectories of the reference point and
     * of the yaw angle are updated in this method.
     * @return true if the motion is ended
     */
    bool step() override;
    
    /*
     * Get the current position and velocity of the center of the object.
     * The velocity of the center of the object is obtained from the velocity
     * of the reference point and the yaw only angular velocity of the object.
     * The position of the center of the object is obtained by integrating the
     * velocity of the center of the object.
     * @param pos yarp::sig::Vector where position is stored
     * @param vel yarp::sig::Vector where velocity is stored
     */
    void getMotion(yarp::sig::Vector &pos,
		   yarp::sig::Vector &vel,
		   yarp::sig::Vector &ref_vel,		   
	           double &yaw) override;    
};

#endif

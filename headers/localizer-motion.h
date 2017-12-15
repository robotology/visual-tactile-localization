/******************************************************************************
 *                                                                            *
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga <nicolapiga@gmail.com>
 */

#ifndef LOCALIZER_MOTION_H
#define LOCALIZER_MOTION_H

#include "headers/unscentedParticleFilter.h"
#include "headers/geometryCGAL.h"
#include "headers/fakePointCloud.h"
#include "headers/motionGenerator.h"

enum class State { init , run, end };
enum class LocalizationType { Static, Motion };

/** 
 *  Results of the algorithm.
 */
struct Results
{
    // real pose
    yarp::sig::Vector real;
    
    // estimated pose    
    yarp::sig::Vector estimate;

    Results() : real(6, 0.0),
	        estimate(6, 0.0) {}
};

/** 
 *  
 */
struct LocalizationPhase
{
    // remember if initialization has been done
    bool initialized;
    
    // displacement from reference point to center
    // of the object
    yarp::sig::Vector displ;

    // whether to hold or not displ 
    // from the previous localization phase settings
    bool holdDisplFromPrevious;
    
    // initial pose of the reference point
    yarp::sig::Vector ref_pos_0;

    // initial yaw angle
    double yaw_0;

    // final pose of the reference point
    yarp::sig::Vector ref_pos_f;

    // final yaw angle
    double yaw_f;

    // step time
    // may be used as a counter for static localization
    double step_time;
    
    // duration of the trajectory
    double duration;

    // number of points of the point cloud
    // used in static localization
    int num_points;

    // type of localization phase
    LocalizationType type;

    // pointer to the point cloud
    FakePointCloud *pc;

    // pointer to motion generator
    MotionGenerator *mg;

    LocalizationPhase(const LocalizationType type,
		      const bool holdDispl = false)
                     : initialized(false),
                       holdDisplFromPrevious(holdDispl),
	               ref_pos_0(3, 0.0),
	               ref_pos_f(3, 0.0),
	               displ(3, 0.0),
	               type(type){ };
};

/** 
 *  This class is the implementation of an Offline UPF based localizer algorithm.
 */
class LocalizerMotion : public yarp::os::RFModule
{
private:
    // resource finder
    yarp::os::ResourceFinder *rf;
    
    // fixed number of contact points per time step
    // for estimation during static phase
    int n_contacts;

    // pointer to Unscented Particle Filter
    UnscentedParticleFilter *upf;

    // current object pose
    yarp::sig::Vector cur_pos;
    yarp::sig::Vector cur_vel;
    yarp::sig::Vector cur_ref_vel;    
    yarp::sig::Vector cur_att;
    double cur_yaw;
    // previous velocity
    yarp::sig::Vector prev_vel;
    yarp::sig::Vector prev_ref_vel;

    // models file names
    std::string model_file_name;
    std::string aux_cloud_1_file_name;
    std::string aux_cloud_2_file_name;

    // fake point cloud engines
    FakePointCloud pc_whole;
    FakePointCloud pc_contacts_1;
    FakePointCloud pc_contacts_2;    

    // static pose generator
    StaticMotionGenerator static_mg;

    // motion generator
    PolynomialMotionGenerator motion_mg;

    // number of steps performed
    int n_steps;    

    // state of the simulation
    State state;

    // vector of localization phases
    std::vector<LocalizationPhase> phases;

    // index of current phase
    int current_phase;

    // results
    std::vector<Results> results;

    /*
     * Load the require parameters using a
     * previously instantiated @see Resource Finder.
     * @param rf a previously instantiated @see Resource Finder
     * @return true/false depending on the outcome
     */
    bool loadParameters(const yarp::os::ResourceFinder &rf);

    /*
     * Save a Polyhedron model as a .OFF (Object File Format) file.
     *
     * @param model a Polyhedron model
     * @param filename the output file name
     * @return true/false depending on the outcome
     */
    bool saveModel(const Polyhedron &model, const std::string &filename);    

    /**
     * Save the real and estimated pose for each time step
     * @param results the results to be saved
     * @return true/false depending on the outcome
     */
    bool saveResults(const std::vector<Results> &results);

    /*
     * Save measurements as a vertices only .OFF (Object File Format) file.
     *
     * @param meas a std::vector<Measure> of Measure measures
     * @param filename the output file name
     * @return true/false depending on the outcome
     */
    bool saveMeas(const std::vector<Measure> &meas, const std::string &filename);

    void saveLocalizationStep(const yarp::sig::Vector &real_pose,
			      const std::vector<Measure> &meas);

    /*
     * Configure the current localization phase.
     *
     * @param current_phase the index of the current phase
     */
    void configureLocPhase(const int &current_phase);
	
    /*
     * Perform localization.
     * param current_phase the index of the current phase
     */
    bool performLocalization(int &current_phase);

public:
    /*
     * Constructor.
     */
    LocalizerMotion() : cur_pos(3, 0.0),
                        cur_vel(3, 0.0),
	                cur_ref_vel(3, 0.0),
                        cur_att(3, 0.0),
	                cur_yaw(0),
                        prev_vel(3, 0.0),
	                prev_ref_vel(3, 0.0),
	                current_phase(0) { };
    /*
     * Configure the module.
     * @param rf a previously instantiated @see ResourceFinder
     */
    bool configure(yarp::os::ResourceFinder &rf) override;

    /*
     * Return the module period.
     */
    double getPeriod() override;
    
    /*
     * Define the behavior of this module.
     */
    bool updateModule() override;

    /*
     * Define the cleanup behavior.
     */
    bool close() override;    
};

#endif

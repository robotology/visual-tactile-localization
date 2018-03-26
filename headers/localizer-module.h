/******************************************************************************
 *                                                                            *
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga <nicolapiga@gmail.com>
 */

#ifndef LOCALIZER_MODULE_H
#define LOCALIZER_MODULE_H

// yarp
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Mutex.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncoders.h>

// icub-main
#include <iCub/iKin/iKinFwd.h>
#include <iCub/skinDynLib/skinContactList.h>

// std
#include <vector>

#include "headers/filterCommand.h"
#include "headers/unscentedParticleFilter.h"
#include "headers/PointCloud.h"

enum class FilteringType { visual, tactile };

struct Data
{
    // ground truth pose
    yarp::sig::Vector ground_truth;

    // estimated pose
    yarp::sig::Vector estimate;

    // current measurements
    std::vector<yarp::sig::Vector> meas;

    // current input
    yarp::sig::Vector input;

    // execution time of filtering step
    double exec_time;

    // type of step, i.e. vision or tactile
    int step_type;

    Data() : ground_truth(6, 0.0),
	     estimate(6, 0.0) {};
};

/**
 *  This class is the implementation of an Online UPF based localizer algorithm.
 */
class LocalizerModule : public yarp::os::RFModule
{
private:
    // resource finder
    yarp::os::ResourceFinder *rf;

   /**
    * Filter
    */

    // pointer to Unscented Particle Filter
    UnscentedParticleFilter upf;

    // system noise covariance matrices
    yarp::sig::Vector Q_vision;
    yarp::sig::Vector Q_tactile;

    // measurement noise variance
    double R_vision;
    double R_tactile;

    // last estimate available
    yarp::sig::Vector last_estimate;
    // last ground truth available
    yarp::sig::Vector last_ground_truth;

    // storage for collection of
    // filtering data
    std::vector<Data> storage;
    bool storage_on;
    yarp::os::Mutex storage_on_mutex;

    // variables required to collect
    // filtering execution time
    double t_i;
    double t_f;
    double exec_time;

    // path where to save output
    std::string output_path;

    // flags
    bool estimate_available;
    bool filtering_enabled;
    bool is_first_step;
    FilteringType filtering_type;

    // period
    double period;

    /*
     */

   /**
    * FrameTransformClient required to
    * - publish the estimate of the filter
    * - retrieve the pose of the robot root frame
    */

    // PolyDriver required to access a yarp::dev::IFrameTransform
    yarp::dev::PolyDriver drv_transform_client;

    // pointer to yarp::dev::IFrameTransform view of the PolyDriver
    yarp::dev::IFrameTransform* tf_client;

    // source and target frame names for estimate
    std::string est_source_frame_name;
    std::string est_target_frame_name;

    // source and target frame names for robot root frame
    std::string robot_source_frame_name;
    std::string robot_target_frame_name;

    // transformation from inertial frame to
    // the root frame of the robot published by gazebo
    // required to convert point clouds in the robot reference frame
    yarp::sig::Matrix inertial_to_robot;

    /*
     */

    /**
     *  IEncoders
     */

    // PolyDriver required to access yarp::dev::IEncoders encoders
    yarp::dev::PolyDriver drv_right_arm;
    yarp::dev::PolyDriver drv_left_arm;
    yarp::dev::PolyDriver drv_torso;

    // pointers to yarp::dev::IEncoders view of the PolyDriver
    yarp::dev::IEncoders *ienc_right_arm;
    yarp::dev::IEncoders *ienc_left_arm;
    yarp::dev::IEncoders *ienc_torso;

    /*
     */

    /**
     *  iCub forward kinematics
     */

    iCub::iKin::iCubArm right_arm_kin;
    iCub::iKin::iCubArm left_arm_kin;

    // middle finger only for now
    // TODO: add the other fingers
    iCub::iKin::iCubFinger right_middle;
    iCub::iKin::iCubFinger left_middle;

    /*
     */

   /**
    * Ports
    */

    // port where new filtering command is received
    yarp::os::BufferedPort<yarp::sig::FilterCommand> port_in;

    // point cloud
    yarp::os::BufferedPort<PointCloud> port_pc;

    // contact points
    yarp::os::BufferedPort<iCub::skinDynLib::skinContactList> port_contacts;

    // rpc server
    yarp::os::RpcServer rpc_port;

    // names
    std::string input_port_name;
    std::string port_pc_name;
    std::string port_contacts_name;
    std::string rpc_port_name;
    /*
     */

    yarp::sig::FilterCommand* cmd;

    /*
     * Read a diagonal matrix from the configuration file
     *
     * @param tag the name of the matrix in the configuration file
     * @param size size of the diagonal
     * @param diag the output diagonal as a vector
     * @return true/false on success/failure
     */
    bool readDiagonalMatrix(const std::string &tag,
			    const int &size,
			    yarp::sig::Vector &diag);
    /*
     * Load the required parameters using a
     * previously instantiated @see Resource Finder.
     * @return true/false depending on the outcome
     */
    bool loadParameters();

    /*
     * Transform a point cloud expressed in the inertial frame
     * to a point cloud expressed with respect to the robot root frame
     * @param pc the input point cloud
     * @param pc_out the transformed point cloud
     */
    void transformPointCloud(const PointCloud& pc,
			     std::vector<yarp::sig::Vector> &pc_out);

    /*
     * Extract finger tips contact points from a skinContactList
     * @param list the input skin contact list
     * @param point_right contact points on the right hand finger tips
     * @param point_right contact points on the left hand finger tips
     */
    void getContactPoints(const iCub::skinDynLib::skinContactList &list,
			  std::vector<yarp::sig::Vector> &points_right,
			  std::vector<yarp::sig::Vector> &points_left);
    /*
     * Evaluate the velocity of a finger using geometric jacobians and
     * angular joints velocities
     * @param hand_name the name of the desired hand
     * @param finger_name the name of the desired finger
     * @param velocity the computed velocity
     */
    bool getFingerVelocity(const std::string &hand_name,
			   const std::string &finger_name,
			   yarp::sig::Vector &velocity);
    /*
     * Process a command coming from the input port
     * @param cmd command to the filtering algorithm
     */
    void processCommand(const yarp::sig::FilterCommand &filter_cmd);

    /*
     * Perform a filtering step
     */
    void performFiltering();

    /*
     * Stops filtering
     */
    void stopFiltering();

    /*
     * Publish the last estimate available
     * over a FrameTransformServer
     */
    void publishEstimate();

    /*
     * Retrieve the ground truth coming from the
     * simulation environment
     * @param pose the ground truth pose of the object
     * @return true/false on sucess
     */
    bool retrieveGroundTruth(yarp::sig::Vector &pose);

    /*
     * Reset internal storage
     */
    void resetStorage();

    /*
     * Store data in the internal storage
     * @param ground_truth the current ground truth
     * @param estimate the current estimate
     * @param meas the current measurements
     * @param input the current input
     * @param ground_truth execution time of last filtering step
     */
    void storeData(const yarp::sig::Vector &ground_truth,
		   const yarp::sig::Vector &estimate,
		   const std::vector<yarp::sig::Vector> &meas,
		   const yarp::sig::Vector &input,
		   const double &exec_time);

    /*
     * Save to a .OFF file a mesh representing a ground truth pose
     * or an estimated pose
     * @param pose vector containing a pose
     * @param file_name where to save the mesh file
     * @return true/false on success
     */
    bool saveMesh(const yarp::sig::Vector &pose,
		  const std::string &file_name);

    /*
     * Save to .OFF file a vertex-only mesh representing measurements
     * @param meas vector containing measurements
     * @param file_name where to save the mesh file
     * @return true/false on success
     */
    bool saveMeas(const std::vector<yarp::sig::Vector> &meas,
		  const std::string &file_name);

    /*
     * Save all the data (ground truth, estimate, input,
     * measurements and meshes) to file.
     * @param data data to be saved
     * @return true/false on success
     */
    bool saveData(const std::vector<Data> &data);

    /*
     * Rpc server callback
     * @param command the command received
     * @param reply the reply from the server
     * @return true/false on success/failure
     */
    bool respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply);

public:
    LocalizerModule() : last_estimate(6, 0.0),
            	        Q_vision(6, 0.0),
	                Q_tactile(6, 0.0) { };
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

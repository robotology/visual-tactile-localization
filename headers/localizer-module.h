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
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/sig/PointCloud.h>


// icub-main
/* #include <iCub/iKin/iKinFwd.h> */
#include <iCub/skinDynLib/skinContactList.h>
#include <iCub/perception/springyFingers.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>

// std
#include <vector>
#include <unordered_map>

#include "headers/FilterCommand.h"
#include "headers/unscentedParticleFilter.h"
#include "headers/PointCloud.h"
#include "headers/fwd_kin_ext.h"

typedef yarp::sig::PointCloud<yarp::sig::DataXYZ> PointCloudXYZ;

enum class FilteringType { visual = 0, tactile = 1 };

struct Data
{
    // ground truth pose
    yarp::sig::Vector ground_truth;

    // estimated pose
    yarp::sig::Vector estimate;

    // particles
    std::vector<yarp::sig::Vector> particles;

    // current measurements
    std::vector<yarp::sig::Vector> meas;

    // current fingers joints angles
    std::unordered_map<std::string, yarp::sig::Vector> fingers_joints;

    // current finger positions
    std::unordered_map<std::string, yarp::sig::Vector> fingers_pos;

    // current finger velocities
    std::unordered_map<std::string, yarp::sig::Vector> fingers_vels;

    // contacts due to tactile perception
    std::unordered_map<std::string, bool> contacts_tactile;

    // contacts detected using springy fingers modeling
    std::unordered_map<std::string, bool> contacts_springy;

    // current input
    yarp::sig::Vector input;

    // time stamp relative to the filtering step
    double time_stamp;

    // execution time of filtering step
    double exec_time;

    // type of data, i.e. vision or tactile
    FilteringType data_type;

    // whether this is the first chunk of a point cloud
    // to be ignored for tactile measurements
    bool is_first_chunk;

    // filtered pc used as input to the filter
    std::vector<yarp::sig::Vector> filtered_pc;

    // true point cloud
    // coming from the stereo vision
    std::vector<yarp::sig::Vector> true_pc;

    Data() : ground_truth(6, 0.0),
             estimate(6, 0.0) {};
};

/**
 *  This class is the implementation of an Online UPF based localizer algorithm.
 */
class LocalizerModule : public yarp::os::RFModule
{
private:
   /**
    * Filter
    */

    // Unscented Particle Filter(s)
    UnscentedParticleFilter upf0;
    UnscentedParticleFilter upf1;

    // system noise covariance matrices
    yarp::sig::Vector Q_vision;
    yarp::sig::Vector Q_tactile;

    // measurement noise variance
    double R_vision;
    double R_tactile;

    // last estimate available
    yarp::sig::Vector last_estimate;
    yarp::sig::Vector last_vis_estimate;
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

    // hand to be used during tactile filtering
    // and contacts probing
    std::string tac_filt_hand_name;

    // robot name
    std::string robot_name;

    // flags
    bool estimate_available;
    bool filtering_enabled;
    bool contacts_probe_enabled;
    bool is_first_step;
    bool is_simulation;
    bool use_analogs;
    bool use_analogs_bounds;
    bool use_ext_vel_observer;
    bool use_springy;
    FilteringType filtering_type;

    // period
    double period;

    // size of chunk used in visual filtering
    int visual_chunk_size;

    // uniform sample parameter used to subsample the point cloud
    bool use_pc_subsampling;
    unsigned int subsample_n_points;

    // random sample parameter used to shuffle the point cloud
    bool use_pc_shuffle;
    double shuffle_resize_factor;

    // parameters for radius outlier removal
    bool use_pc_outlier_rem;
    bool use_pc_dense_outlier_rem;
    double outlier_rem_radius;
    int outlier_rem_neigh;
    double dense_outlier_rem_thr;

    // fingers to be excluded from contact detection
    std::vector<std::string> excluded_fingers;

    // visual tactile mismatch
    yarp::sig::Matrix vis_tac_mismatch;

    /*
     */

   /**
    * FrameTransformClient required to
    * - publish the estimate of the filter
    * - retrieve the ground truth
    * - retrieve the pose of the robot root frame (simulation only)
    */

    // PolyDriver required to access a yarp::dev::IFrameTransform
    yarp::dev::PolyDriver drv_transform_client;

    // pointer to yarp::dev::IFrameTransform view of the PolyDriver
    yarp::dev::IFrameTransform* tf_client;

    // source and target frame names for estimate
    std::string est_source_frame_name;
    std::string est_target_frame_name;

    // source and target frame names for robot root frame (simulation only)
    std::string robot_source_frame_name;
    std::string robot_target_frame_name;

    // source and target frame names for ground truth
    std::string gtruth_source_frame_name;
    std::string gtruth_target_frame_name;

    // transformation from inertial frame to
    // the root frame of the robot published by gazebo
    // required to convert point clouds in the robot reference frame
    yarp::sig::Matrix inertial_to_robot;

    /*
     */

    /**
     *  IEncoders, IAnalogs and IControlLimits
     */

    // PolyDriver required to access yarp::dev::IEncoders encoders
    yarp::dev::PolyDriver drv_right_arm;
    yarp::dev::PolyDriver drv_left_arm;
    yarp::dev::PolyDriver drv_torso;
    yarp::dev::PolyDriver drv_right_analog;
    yarp::dev::PolyDriver drv_left_analog;

    // pointers to yarp::dev::IEncoders view of the PolyDriver
    yarp::dev::IEncoders *ienc_right_arm;
    yarp::dev::IEncoders *ienc_left_arm;
    yarp::dev::IEncoders *ienc_torso;
    yarp::dev::IAnalogSensor *ianalog_right;
    yarp::dev::IAnalogSensor *ianalog_left;

    // control limits
    yarp::dev::IControlLimits *ilim_right;
    yarp::dev::IControlLimits *ilim_left;

    // matrix of analog bounds
    // for encoders of proximal/distal joints
    yarp::sig::Matrix analog_bounds;

    /*
     */

    /**
     *  iCub forward kinematics
     */

    iCub::iKin::iCubArm right_arm_kin;
    iCub::iKin::iCubArm left_arm_kin;

    // fingers
    std::vector<std::string> fingers_names;
    std::unordered_map<std::string, iCub::iKin::iCubFingerExt> fingers_kin;

    // springy fingers for contact detection
    iCub::perception::SpringyFingersModel right_springy_fingers;
    iCub::perception::SpringyFingersModel left_springy_fingers;
    yarp::sig::Vector springy_thres_right;
    yarp::sig::Vector springy_thres_left;

    // linear estimator for estimation of joints velocities
    // in case joints speeds are not available from the robot
    std::unique_ptr<iCub::ctrl::AWLinEstimator> joints_vel_estimator;

    /*
     */

   /**
    * Ports
    */

    // port where new filtering command is received
    yarp::os::BufferedPort<yarp::sig::FilterCommand> port_in;

    // point cloud
    // used in simulation with Gazebo
    yarp::os::BufferedPort<PointCloud> port_pc_sim;
    // used with real robot
    yarp::os::BufferedPort<PointCloudXYZ> port_pc;
    /* yarp::os::BufferedPort<PointCloudXYZ> port_filtered_pc; */

    // contact points
    // used in simulation (GazeboYarpSkin)
    yarp::os::BufferedPort<iCub::skinDynLib::skinContactList> port_contacts_sim;
    // used with real robot
    yarp::os::BufferedPort<yarp::sig::Vector> port_contacts;

    // velocityObserver ports
    /* yarp::os::BufferedPort<yarp::sig::Vector> ext_vel_obs_torso; */
    /* yarp::os::BufferedPort<yarp::sig::Vector> ext_vel_obs_left_arm; */
    /* yarp::os::BufferedPort<yarp::sig::Vector> ext_vel_obs_right_arm; */

    // rpc server
    yarp::os::RpcServer rpc_port;
    yarp::os::Mutex mutex;

    // names
    std::string input_port_name;
    std::string port_pc_name;
    std::string port_filtered_pc_name;
    std::string port_contacts_name;
    std::string rpc_port_name;
    /*
     */

    yarp::sig::FilterCommand* cmd;

    /*
     * Read a diagonal matrix from the configuration file
     *
     * @param rf a previously instantiated @see ResourceFinder
     * @param tag the name of the matrix in the configuration file
     * @param size size of the diagonal
     * @param diag the output diagonal as a vector
     * @return true/false on success/failure
     */
    bool readDiagonalMatrix(const yarp::os::ResourceFinder &rf,
                            const std::string &tag,
                            const int &size,
                            yarp::sig::Vector &diag);

    bool loadListDouble(yarp::os::ResourceFinder &rf,
                        const std::string &key,
                        const int &size,
                        yarp::sig::Vector &list);

    bool loadListStrings(const yarp::os::ResourceFinder &rf,
                         const std::string &tag_name,
                         std::vector<std::string> &list);
    /*
     * Load the required parameters using a
     * previously instantiated @see Resource Finder.
     * @param rf a previously instantiated @see ResourceFinder
     * @return true/false depending on the outcome
     */
    bool loadParameters(yarp::os::ResourceFinder &rf);

    /*
     * Get a point cloud from the simulation environment
     *
     * @param pc_out the transformed point cloud
     * @return true/false on success/failure
     */
    bool getPointCloudSim(std::vector<yarp::sig::Vector> &pc);

    /*
     * Subsample a point cloud by skipping n points.
     *
     * @param pc_in the input point cloud
     * @param pc_out the subsampled point cloud
     * @param skip_points the value of the variable n
     * @return true/false on success/failure
     */
    void subsamplePointCloud(const std::vector<yarp::sig::Vector> &pc_in,
                             std::vector<yarp::sig::Vector> &pc_out,
                             const unsigned int &skip_points);

    /*
     * Shuffle a point cloud by extracting indexes
     * according to a uniform distribution.
     *
     * The operation is completed when resize_factor * size
     * points have been sampled, where size is the total size
     * of the point cloud.
     *
     * @param pc_in the input point cloud
     * @param pc_out the subsampled point cloud
     * @param resize_factor the value of the resize factor
     */
    void shufflePointCloud(const std::vector<yarp::sig::Vector> &pc_in,
                           std::vector<yarp::sig::Vector> &pc_out,
                           const double &resize_factor);

    /*
     * Perform radius outlier removal on the input point cloud.
     *
     * @param pc_in the input point cloud
     * @param pc_out the subsampled point cloud
     * @param radius the radius used in the radius outlier removal
     * @param num_points the number of neighborhoods used in the radius outlier removal
     */
    void removeOutliersFromPointCloud(const std::vector<yarp::sig::Vector> &pc_in,
                                      std::vector<yarp::sig::Vector> &pc_out,
                                      const double &radius, const int num_points);

    void removeDenseOutliersFromPointCloud(const double &threshold,
                                           const std::vector<yarp::sig::Vector> &pc_in,
                                           std::vector<yarp::sig::Vector> &pc_out);
    /*
     * Get a point cloud from the stereo vision setup.
     *
     * @param pc the point cloud
     * @return true/false on success/failure
     */
    bool getPointCloud(std::vector<yarp::sig::Vector> &pc);

    /*
     * Setup the analog bounds for
     * encoders of proximal/distal joints
     *
     */
    void setupAnalogBounds();

    /*
     * Extract finger tips contact points from a skinContactList
     *
     * To be used in simulation.
     *
     * @param hand_name name of the hand to be used
     * @param points contact points on finger tips of the specified hand
     */
    /* bool getContactPointsSim(const std::string &hand_name, */
    /*                          std::vector<yarp::sig::Vector> &points); */

    /*
     * Extract finger tips contact points from a map containing position
     * of finger tips and a map containing finger tips contact states
     *
     * @param fingers_contacts map between fingers names and finger tips contact states
     * @param fingers_pos map between fingers names and figner tips position
     * @param points contact points on finger tips
     */
    void getContactPoints(const std::unordered_map<std::string, bool> &fingers_contacts,
                          const std::unordered_map<std::string, yarp::sig::Vector> &fingers_pos,
                          std::unordered_map<std::string, yarp::sig::Vector> &contact_points);

    /*
     * Extract finger tips contacts state from a yarp::sig::Vector
     * coming from SkinManager on the real robot
     *
     * @param hand_name name of the hand to be used
     * @param points contact states on finger tips of the specified hand
     */
    bool getContacts(const std::string &hand_name,
                     std::unordered_map<std::string, bool> &contacts);

    /*
     * Extract finger tips contacts state and contact points from skinContactList
     *
     * To be used in simulation.
     *
     * @param hand_name name of the hand to be used
     * @param points contact states on finger tips of the specified hand
     */
    bool getContactsSim(const std::string &hand_name,
                        std::unordered_map<std::string, bool> &contacts,
                        std::unordered_map<std::string, yarp::sig::Vector> &contact_points);

    bool getContactsSpringy(const std::string &hand_name,
                            std::unordered_map<std::string, bool> &contacts);

    /*
     * Extract arm and torso angles and angular rates.
     * @param arm_name the name of the desired arm
     * @param arm_angles vector of arm angles
     * @param torso_angles vector of torso angles
     * @param arm_ang_rates vector of arm angular rates
     * @param torso_ang_rates vector of torso angular rates
     * @param fingers_analogs vector of fingers proximal/distal joints angles
     */
    bool getChainJointsState(const std::string &arm_name,
                             yarp::sig::Vector &arm_angles,
                             yarp::sig::Vector &torso_angles,
                             yarp::sig::Vector &arm_ang_rates,
                             yarp::sig::Vector &torso_ang_rates,
                             yarp::sig::Vector &fingers_analogs);

    /*
     * Extract angles and angular rates for the hand
     * so that they are compatible with the class iCub::iKin::iCubArm
     * @param hand_name the name of the desired hand
     * @param arm_angles vector of arm angles
     * @param torso_angles vector of torso angles
     * @param arm_ang_rates vector of arm angular rates
     * @param torso_ang_rates vector of torso angular rates
     * @param hand_angles vector of hand angles
     * @param hand_ang_rates vector of hand angular rates
     */
    void getHandJointsState(const std::string &hand_name,
                            const yarp::sig::Vector &arm_angles,
                            const yarp::sig::Vector &torso_angles,
                            const yarp::sig::Vector &arm_ang_rates,
                            const yarp::sig::Vector &torso_ang_rates,
                            yarp::sig::Vector &hand_angles,
                            yarp::sig::Vector &hand_ang_rates);

    /*
     * Evaluate the the twist of the desired hand.
     * @param hand_name the name of the desired hand
     * @param hand_angles vector of hand angles
     * @param hand_ang_rates vector of hand angular rates
     * @param lin_velocity linear velocity of the center of the hand
     * @param ang_velocity angular velocity of the hand
     */
    void getHandTwist(const std::string &hand_name,
                      const yarp::sig::Vector hand_angles,
                      const yarp::sig::Vector hand_ang_rates,
                      yarp::sig::Vector &lin_velocity,
                      yarp::sig::Vector &ang_velocity);

    /*
     * Update the chain of the specified finger.
     * @param hand_name the name of the desired hand
     * @param finger_name the name of the desired finger
     * @param finger_angles vector of finger joints angles
     */
    void updateFingerConfiguration(const std::string &hand_name,
                                   const std::string &finger_name,
                                   const yarp::sig::Vector &finger_angles);
    /*
     * Extract angles and angular rates for the finger
     * so that they are compatible with the class iCub::iKin::iCubFinger
     * @param hand_name the name of the desired hand
     * @param finger_name the name of the desired finger
     * @param arm_angles vector of arm angles
     * @param arm_ang_rates vector of arm angular rates
     * @param finger_analogs vector of fingers proximal/distal joints angles
     * @param hand_angles vector of hand angles
     * @param hand_ang_rates vector of hand angular rates
     */
    void getFingerJointsState(const std::string &hand_name,
                              const std::string &finger_name,
                              const yarp::sig::Vector &arm_angles,
                              const yarp::sig::Vector &arm_ang_rates,
                              const yarp::sig::Vector &fingers_analogs,
                              yarp::sig::Vector &finger_angles,
                              yarp::sig::Vector &finger_ang_rates);
    /*
     * Get the velocity of the finger tip relative to the center of the hand
     * and expressed in the hand root frame.
     * @param finger_name the name of the desired finger
     * @param hand_name the name of the desired hand
     * @param finger_angles vector of finger joints angles
     * @param finger_ang_rates vector of finger joints angular rates
     * @param velocity the evaluated velocity
     */
    void getFingerRelativeVelocity(const std::string &finger_name,
                                   const std::string &hand_name,
                                   const yarp::sig::Vector &finger_angles,
                                   const yarp::sig::Vector &finger_ang_rates,
                                   yarp::sig::Vector &velocity);
    /*
     * Get the position of the finger with respect to the robot root frame
     * expressed in the robot root frame.
     * @param finger_name the name of the desired finger
     * @param hand_name the name of the desired hand
     * @param hand_pos position of the origin of the hand root frame
     * @param hand_att attitude of the hand with respect to the robot root frame
     * @param position the evaluated position
     */
    void getFingerPosition(const std::string &finger_name,
                           const std::string &hand_name,
                           const yarp::sig::Vector &hand_pos,
                           const yarp::sig::Matrix &hand_att,
                           yarp::sig::Vector &position);
    /*
     * Get the total velocity of the finger tip expressed in the robot root frame.
     * @param finger_name the name of the desired finger
     * @param hand_name the name of the desired hand
     * @param hand_lin_vel the linear velocity of center of the hand
     * @param hand_ang_vel the angular velocity of the hand
     * @param finger_angles vector of finger joints angles
     * @param finger_ang_rates vector of finger joints angular rates
     * @param hand_2_robot rotation matrix between the robot root frame
     *        and the hand root frame.
     * @param velocity the evaluated velocity
     */
    void getFingerVelocity(const std::string &finger_name,
                           const std::string &hand_name,
                           const yarp::sig::Vector &hand_lin_vel,
                           const yarp::sig::Vector &hand_ang_vel,
                           const yarp::sig::Vector &finger_angles,
                           const yarp::sig::Vector &finger_ang_rates,
                           const yarp::sig::Matrix &hand_2_robot,
                           yarp::sig::Vector &velocity);
    /*
     * Get data related to fingers required for filtering
     * @param hand_name the name of the desired hand
     * @param angles map between the finger names and the
     *        joints angles of the fingers
     * @param angles map between the finger names and the
     *        positions of the fingers
     * @param lin_vels map between the finger names and the
     *        linear velocities of the fingers
     */
    void getFingersData(const std::string &hand_name,
                        std::unordered_map<std::string, yarp::sig::Vector> &angles,
                        std::unordered_map<std::string, yarp::sig::Vector> &positions,
                        std::unordered_map<std::string, yarp::sig::Vector> &lin_vels);

    bool calibrateHand(const std::string &hand_name);

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
     * Perform probe of contacts on finger tips
     */
    void performContactsProbe();

    /*
     * Stops filtering
     */
    void stopFiltering();

    /*
     * Initializer filters
     */
    void initFilters();

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

    void evaluateVisualTactileMismatch(const yarp::sig::Vector &visual_estimate,
                                       const yarp::sig::Vector &tactile_estimate,
                                       yarp::sig::Matrix &mismatch);
    /*
     * Reset internal storage
     */
    void resetStorage();

    /*
     * Store data in the internal storage
     * @param data_type the type of data, i.e. relative to visual or tactile filtering
     * @param ground_truth the current ground truth
     * @param estimate the current estimate
     * @param particles the particles in the current state
     * @param meas the current measurements
     * @param input the current input
     * @param ground_truth execution time of last filtering step
     * @return a reference to the data set stored
     */
    Data& storeData(const FilteringType &data_type,
                    const yarp::sig::Vector &ground_truth,
                    const yarp::sig::Vector &estimate,
                    const std::vector<yarp::sig::Vector> &particles,
                    const std::vector<yarp::sig::Vector> &meas,
                    const yarp::sig::Vector &input,
                    const double &time_stamp,
                    const double &exec_time);
    /*
     * Store data in the internal storage for the visual filtering case
     * @param data_type the type of data, i.e. relative to visual or tactile filtering
     * @param is_first_chunk whether this is the first chunk of a point cloud
     * @param ground_truth the current ground truth
     * @param estimate the current estimate
     * @param particles the particles in the current state
     * @param meas the current measurements
     * @param input the current input
     * @param filtered_pc the effective pc used by the filter
     * @param true_pc the true pc from the stereo vision
     * @param ground_truth execution time of last filtering step
     * @return a reference to the data set stored
     */
    void storeDataVisual(const FilteringType &data_type,
			 const bool &is_first_chunk,
			 const yarp::sig::Vector &ground_truth,
			 const yarp::sig::Vector &estimate,
                         const std::vector<yarp::sig::Vector> &particles,
			 const std::vector<yarp::sig::Vector> &meas,
			 const yarp::sig::Vector &input,
			 const std::vector<yarp::sig::Vector> &filtered_pc,
			 const std::vector<yarp::sig::Vector> &true_pc,
			 const double &time_stamp,
			 const double &exec_time);

    /*
     * Store data in the internal storage for the tactile filtering case
     * @param ground_truth the current ground truth
     * @param estimate the current estimate
     * @param particles the particles in the current state
     * @param meas the current measurements
     * @param input the current input
     * @param fingers_joints the current joints configuration of the fingers
     * @param fingers_pos the current positions of the fingers
     * @param fingers_vels the current linear velocities of the fingers
     * @param contacts_tactile contacts due to tactile perception
     * @param contacts_springy contacts detected using springy fingers modeling
     * @param exec_time_truth execution time of last filtering step
     */
    void storeDataTactile(const yarp::sig::Vector &ground_truth,
                          const yarp::sig::Vector &estimate,
                          const std::vector<yarp::sig::Vector> &particles,
                          const std::vector<yarp::sig::Vector> &meas,
                          const yarp::sig::Vector &input,
                          std::unordered_map<std::string, yarp::sig::Vector> fingers_joints,
                          std::unordered_map<std::string, yarp::sig::Vector> fingers_pos,
                          std::unordered_map<std::string, yarp::sig::Vector> fingers_vels,
                          std::unordered_map<std::string, bool> contacts_tactile,
                          std::unordered_map<std::string, bool> contacts_springy,
                          const double &time_stamp,
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
     * Save to a .CSV file the joints angles of all the fingers.
     * @param angles a map between the fingers names and the joints angles
     * @param file_name where to save the file
     * @return true/false on success
     */
    bool saveFingersJoints(const std::unordered_map<std::string, yarp::sig::Vector> &angles,
                           const std::string &file_name);

    /*
     * Save to a .CSV file the positions of all the fingers.
     * @param positions a map between the fingers names and positions of the fingers
     * @param file_name where to save the file
     * @return true/false on success
     */
    bool saveFingersPositions(const std::unordered_map<std::string, yarp::sig::Vector> &positions,
                              const std::string &file_name);

    /*
     * Save to a .CSV file the linear velocities of all the fingers.
     * @param velocities a map between the fingers names and the velocities of the fingers
     * @param file_name where to save the file
     * @return true/false on success
     */
    bool saveFingersVelocities(const std::unordered_map<std::string, yarp::sig::Vector> &velocities,
                               const std::string &file_name);

    /*
     * Save to a .CSV file the contacts detected.
     * @param contacts_tactile contacts due to tactile perception
     * @param contacts_springy contacts detected using springy fingers modeling
     * @param file_name where to save the file
     * @return true/false on success
     */
    bool saveContacts(const std::unordered_map<std::string, bool> &contacts_tactile,
                      const std::unordered_map<std::string, bool> &contacts_springy,
                      const std::string &file_name);


    /*
     * Save to a .CSV file all the particles.
     * @param particles the particles to be saved
     * @file_name where to save the file
     * @return true/false on success
     */
    bool saveParticles(const std::vector<yarp::sig::Vector> &particles,
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

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

// icub-main
#include <iCub/iKin/iKinFwd.h>
#include <iCub/skinDynLib/skinContactList.h>

// std
#include <vector>
#include <unordered_map>

#include "headers/FilterCommand.h"
#include "headers/unscentedParticleFilter.h"
#include "headers/PointCloud.h"

enum class FilteringType { visual = 0, tactile = 1 };

struct Data
{
    // ground truth pose
    yarp::sig::Vector ground_truth;

    // estimated pose
    yarp::sig::Vector estimate;

    // current measurements
    std::vector<yarp::sig::Vector> meas;

    // current fingers joints angles
    std::unordered_map<std::string, yarp::sig::Vector> fingers_joints;

    // current finger positions
    std::unordered_map<std::string, yarp::sig::Vector> fingers_pos;

    // current finger velocities
    std::unordered_map<std::string, yarp::sig::Vector> fingers_vels;

    // current input
    yarp::sig::Vector input;

    // time stamp relative to the filtering step
    double time_stamp;

    // execution time of filtering step
    double exec_time;

    // type of data, i.e. vision or tactile
    FilteringType data_type;

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

    // hand to be used during tactile filtering
    std::string tac_filt_hand_name;

    // robot name
    std::string robot_name;

    // flags
    bool estimate_available;
    bool filtering_enabled;
    bool is_first_step;
    bool is_simulation;
    bool use_analogs;
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
     *  IEncoders and IAnalogs
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

    /*
     */

    /**
     *  iCub forward kinematics
     */

    iCub::iKin::iCubArm right_arm_kin;
    iCub::iKin::iCubArm left_arm_kin;

    // fingers
    std::vector<std::string> fingers_names;
    std::unordered_map<std::string, iCub::iKin::iCubFinger> fingers_kin;

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
    // used in simulation (GazeboYarpSkin)
    yarp::os::BufferedPort<iCub::skinDynLib::skinContactList> port_contacts_sim;
    // used with real robot
    yarp::os::BufferedPort<yarp::sig::Vector> port_contacts;

    // rpc server
    yarp::os::RpcServer rpc_port;
    yarp::os::Mutex mutex;

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
    /*
     * Load the required parameters using a
     * previously instantiated @see Resource Finder.
     * @param rf a previously instantiated @see ResourceFinder
     * @return true/false depending on the outcome
     */
    bool loadParameters(yarp::os::ResourceFinder &rf);

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
     *
     * To be used in simulation.
     *
     * @param hand_name name of the hand to be used
     * @param points contact points on finger tips of the specified hand
     */
    bool getContactPointsSim(const std::string &hand_name,
                             std::vector<yarp::sig::Vector> &points);

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
                          std::vector<yarp::sig::Vector> points);

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
     * @param data_type the type of data, i.e. relative to visual or tactile filtering
     * @param ground_truth the current ground truth
     * @param estimate the current estimate
     * @param meas the current measurements
     * @param input the current input
     * @param ground_truth execution time of last filtering step
     * @return a reference to the data set stored
     */
    Data& storeData(const FilteringType &data_type,
                    const yarp::sig::Vector &ground_truth,
                    const yarp::sig::Vector &estimate,
                    const std::vector<yarp::sig::Vector> &meas,
                    const yarp::sig::Vector &input,
                    const double &time_stamp,
                    const double &exec_time);

    /*
     * Store data in the internal storage for the tactile filtering case
     * @param ground_truth the current ground truth
     * @param estimate the current estimate
     * @param meas the current measurements
     * @param input the current input
     * @param fingers_joints the current joints configuration of the fingers
     * @param fingers_pos the current positions of the fingers
     * @param fingers_vels the current linear velocities of the fingers
     * @param exec_time_truth execution time of last filtering step
     */
    void storeDataTactile(const yarp::sig::Vector &ground_truth,
                          const yarp::sig::Vector &estimate,
                          const std::vector<yarp::sig::Vector> &meas,
                          const yarp::sig::Vector &input,
                          std::unordered_map<std::string, yarp::sig::Vector> fingers_joints,
                          std::unordered_map<std::string, yarp::sig::Vector> fingers_pos,
                          std::unordered_map<std::string, yarp::sig::Vector> fingers_vels,
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

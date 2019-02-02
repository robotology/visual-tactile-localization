#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

// yarp
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/math/Math.h>

// icub-main
#include <iCub/iKin/iKinFwd.h>

// std
#include <string>

class ArmController
{
protected:
    // drivers
    yarp::dev::PolyDriver drv_cart;
    yarp::dev::PolyDriver drv_enc_arm;

    // views
    yarp::dev::ICartesianControl *icart;
    yarp::dev::IEncoders *ienc_arm;
    yarp::dev::IEncoders *ienc_torso;

    // cartesian controller initial context
    int startup_cart_context;

    // cartesian controller current context
    int curr_cart_context;
    
    // home pose
    yarp::sig::Vector home_pos;
    yarp::sig::Vector home_att;

    // hand attitude
    yarp::sig::Vector hand_attitude;

    // string that indicates which arm
    // this controller uses
    std::string arm_name;

    // whether the finger tip is attached to the chain or not
    bool is_tip_attached;
    
public:
    
    /*
     * Configure the arm.
     * @param arm_name which arm to be used, right or left
     * @return true/false on success/fail
     */
    bool configure(const std::string port_prefix,
                   const std::string robot_name,
                   const std::string arm_name);

    /*
     * Stop the controller, restore the startup context,
     * close all the drivers.
     */
    void close();

    /*
     * Return a pointer to the Cartesian Controller
     * @return a pointer to the Cartesian Controller
     */
    yarp::dev::ICartesianControl* cartesian();

    /*
     * This function store internally the attitude of the hand
     * required during a pushing phase. 
     *
     * If the arguments yaw, pitch and roll are left to the default
     * values the right (left) hand is placed with the axis orthogonal 
     * to the palm pointing left (right) and the tumb pointing upward.
     *
     * Additional arguments allow to change the default attitude by adding
     * a yaw rotation, with respect to the z axis of the robot waist, 
     * followed by a pitch rotation along the axis orthogonal to the palm hand
     * followed by a roll rotation along the axis parallel to the fingers when 
     * the hand is open.
     * 
     * @param yaw the amount of yaw rotation in degrees
     * @param pitch the amount of pitch rotation in degrees
     * @param roll the amount of roll rotation in degrees
     */
    void setHandAttitude(const double &yaw,
                         const double &pitch,
                         const double &roll);
    /*
     * Get the pose of the frame attached to palm of the hand.
     * @param pos position of the frame
     * @param rot rotation representing attitude of the frame
     * with respect to the robot root frame
     * @return true/false on success/failure
     */
    bool getHandPose(yarp::sig::Vector& pos,
                     yarp::sig::Matrix& rot);

    /*
     * This function attach a tip to the end effector
     * so that the controlled point becomes one of finger of the hand.
     * @param finger_name the name of the finger
     * @return true/false on success/failure
     */
    bool attachFingerTip(const std::string& finger_name);

    /*
     * This function remove the tool tip added to the end effector
     * using the function useFinger.
     * @return true/false on success/failure
     */
    bool detachFingerTip();

    /*
     * Enable use of the torso.
     * @return true/false on success/failure
     */
    bool enableTorso();

    /*
     * Limit the maximum pitch of the torso.
     * @param max_pitch maximum allowed pitch for the torso
     * @return true/false on success/failure
     */
    bool limitTorsoPitch(const double &max_pitch);

    /*
     * Limit the maximum absolute yaw of the torso.
     * @param max maximum absolute allowed yaw for the torso
     * @return true/false on success/failure
     */
    bool limitTorsoYaw(const double &max);

    /*
     * Call the method goToPoseSync of the underlying cartesian controller
     * using the given position and the hand attitude stored in this->
     * hand_attitude.
     */
    void goToPos(const yarp::sig::Vector &pos);

    /*
     * Store the current context of the cartesian controller
     *
     */
    void storeContext();

    /*
     * Restore the previously saved context of the cartesian controller
     *
     */
    void restoreContext();
};

#endif

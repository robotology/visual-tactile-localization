
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
#include <yarp/os/all.h>
#include <yarp/math/Math.h>
#include <yarp/os/Vocab.h>
#include <yarp/math/FrameTransform.h>
#include <yarp/os/Bottle.h>

// CGAL
#include <CGAL/IO/Polyhedron_iostream.h>

// std
#include <fstream>

//
#include "headers/localizer-module.h"
#include "headers/FilterCommand.h"
#include "headers/unscentedParticleFilter.h"
#include "headers/geometryCGAL.h"

using namespace yarp::math;

bool LocalizerModule::readDiagonalMatrix(const yarp::os::ResourceFinder &rf,
                                         const std::string &tag,
                                         const int &size,
                                         yarp::sig::Vector &diag)
{
    // read values of the diagonal
    yarp::os::Bottle *b = rf.find(tag.c_str()).asList();

    if (b == NULL)
        return false;

    if (b->size() < size)
        return false;

    diag.resize(size, 0.0);
    for(size_t i; i < size; i++)
        diag[i] = b->get(i).asDouble();

    return true;
}

bool LocalizerModule::loadParameters(yarp::os::ResourceFinder &rf)
{
    is_simulation = rf.find("simulationMode").asBool();
    if (rf.find("simulationMode").isNull())
        is_simulation = false;
    yInfo() << "Localizer module: simulation mode" << is_simulation;

    est_source_frame_name = rf.find("estimateSourceFrame").asString();
    if (rf.find("estimateSourceFrame").isNull())
        est_source_frame_name = "/iCub/frame";
    yInfo() << "Localizer module: estimate source frame name is" << est_source_frame_name;

    est_target_frame_name = rf.find("estimateTargetFrame").asString();
    if (rf.find("estimateTargetFrame").isNull())
        est_target_frame_name = "/estimate/frame";
    yInfo() << "Localizer module: estimate target frame name is" << est_target_frame_name;

    robot_source_frame_name = rf.find("robotSourceFrame").asString();
    if (rf.find("robotSourceFrame").isNull())
        robot_source_frame_name = "/inertial";
    yInfo() << "Localizer module: robot source frame name is" << robot_source_frame_name;

    robot_target_frame_name = rf.find("robotTargetFrame").asString();
    if (rf.find("robotTargetFrame").isNull())
        robot_target_frame_name = "/iCub/frame";
    yInfo() << "Localizer module: robot target frame name is" << robot_target_frame_name;

    input_port_name = rf.find("inputPort").asString();
    if (rf.find("inputPort").isNull())
        input_port_name = "/upf-localizer:i";
    yInfo() << "Localizer module: input port name is" << input_port_name;

    rpc_port_name = rf.find("rpcServerPort").asString();
    if (rf.find("rpcServerPort").isNull())
        rpc_port_name = "/upf-localizer/service";
    yInfo() << "Localizer module: rpc server port name is" << rpc_port_name;

    port_pc_name = rf.find("pointCloudInputPort").asString();
    if (rf.find("pointCloudInputPort").isNull())
        port_pc_name = "/upf-localizer/pc:i";
    yInfo() << "Localizer module: point cloud input port name is" << port_pc_name;

    port_contacts_name = rf.find("contactsInputPort").asString();
    if (rf.find("contactsInputPort").isNull())
        port_contacts_name = "/upf-localizer/contacts:i";
    yInfo() << "Localizer module: contact points input port name is" << port_contacts_name;

    if (!rf.check("outputPath"))
    {
        yError() << "Localizer module: output path not provided!";
        return false;
    }
    output_path = rf.findFile("outputPath");
    yInfo() << "Localizer module: output path is" << output_path;

    if (!readDiagonalMatrix(rf, "visionQ", 6, Q_vision))
    {
        // set default value for covariance matrix
        Q_vision.setSubvector(0, yarp::sig::Vector(3, 0.0001));
        Q_vision.setSubvector(3, yarp::sig::Vector(3, 0.01));
    }
    yInfo() << "Localizer module: Q matrix for vision is" << Q_vision.toString();

    if (!readDiagonalMatrix(rf, "tactileQ", 6, Q_tactile))
    {
        Q_tactile[0] = 0.00001;
        Q_tactile[1] = 0.00001;
        Q_tactile[2] = 0.00000001;
        Q_tactile[3] = 0.01;
        Q_tactile[4] = 0.000001;
        Q_tactile[5] = 0.000001;
    }
    yInfo() << "Localizer module: Q matrix for tactile is" << Q_tactile.toString();

    R_vision = rf.find("visionR").asDouble();
    if (rf.find("visionR").isNull())
        R_vision = 0.0001;
    yInfo() << "Localizer module: R for vision is" << R_vision;

    R_tactile = rf.find("tactileR").asDouble();
    if (rf.find("tactileR").isNull())
        R_tactile = 0.0001;
    yInfo() << "Localizer module: R for tactile is" << R_tactile;

    period = rf.find("period").asDouble();
    if (rf.find("period").isNull())
        period = 0.01;
    yInfo() << "Localizer module: period " << period;

    return true;
}

void LocalizerModule::transformPointCloud(const PointCloud& pc,
                                          std::vector<yarp::sig::Vector> &pc_out)
{
    // copy data to pc_out
    for (size_t i=0; i<pc.size(); i++)
    {
        PointCloudItem item = pc[i];
        yarp::sig::Vector point(3, 0.0);
        point[0] = item.x;
        point[1] = item.y;
        point[2] = item.z;

        pc_out.push_back(point);
    }

    // transform the points taking into account
    // the root link of the robot
    for (size_t i=0; i<pc_out.size(); i++)
    {
        yarp::sig::Vector point(4, 0.0);
        point.setSubvector(0, pc_out[i]);
        point[3] = 1;

        // transform the point so that
        // it is relative to the orign of the robot root frame
        // and expressed in the robot root frame
        point = SE3inv(inertial_to_robot) * point;

        pc_out[i] = point.subVector(0,2);
    }
}

bool LocalizerModule::getContactPointsSim(const std::string &hand_name,
                                          std::vector<yarp::sig::Vector> &points)
{
    // check if new data is available
    iCub::skinDynLib::skinContactList *list;
    list = port_contacts_sim.read(false);
    if (list == NULL)
        return false;

    // clear vector
    points.clear();

    // extract contacts coming from finger tips only
    for (size_t i=0; i<list->size(); i++)
    {
        // extract the skin contact
        const iCub::skinDynLib::skinContact &skin_contact = (*list)[i];

        // need to verify if this contact was effectively produced
        // by taxels on the finger tips
        // in order to simplify things the Gazebo plugin only sends one
        // taxel id that is used to identify which finger is in contact
        std::vector<unsigned int> taxels_ids = skin_contact.getTaxelList();
        // taxels ids for finger tips are between 0 and 59
        if (taxels_ids[0] >= 0 && taxels_ids[0] < 60)
        {
            if ((hand_name == "right") &&
                (skin_contact.getSkinPart() == iCub::skinDynLib::SkinPart::SKIN_RIGHT_HAND))
                points.push_back(skin_contact.getGeoCenter());
            else if ((hand_name == "left") &&
                     (skin_contact.getSkinPart() == iCub::skinDynLib::SkinPart::SKIN_LEFT_HAND))
                points.push_back(skin_contact.getGeoCenter());
        }
    }

    return true;
}

void LocalizerModule::getContactPoints(const std::unordered_map<std::string, bool> &fingers_contacts,
				       const std::unordered_map<std::string, yarp::sig::Vector> &fingers_pos,
				       std::vector<yarp::sig::Vector> points)
{
    // clear vector
    points.clear();

    for (auto it = fingers_contacts.begin(); it != fingers_contacts.end(); it++)
    {
	const std::string &finger_name = it->first;
	const bool &is_contact = it->second;

	// if this finger is in contact
	// add the contact point to the list
	if (is_contact)
	    points.push_back(fingers_pos.at(finger_name));
    }
}

bool LocalizerModule::getContacts(const std::string &hand_name,
                                  std::unordered_map<std::string, bool> &contacts)
{
    contacts.clear();

    // reset finger_contacts
    contacts["thumb"] = false;
    contacts["index"] = false;
    contacts["middle"] = false;
    contacts["ring"] = false;
    contacts["little"] = false;

    // try to read skin data from the port
    yarp::sig::Vector *skin_data = port_contacts.read(false);
    if (skin_data == NULL)
	return false;

    // size should be 192 for hand data
    if (skin_data->size() != 192)
	return false;

    // finger tips taxels are in the range 0-59
    double thr = 0;
    std::string finger_name;
    for (size_t i=0; i<60; i++)
    {
	if ((*skin_data)[i] > thr)
	{
	    if (i >=0 && i < 12)
		finger_name = "index";
	    else if (i >= 12 && i < 24)
		finger_name = "middle";
	    else if (i >= 24 && i < 36)
		finger_name = "ring";
	    else if (i >= 36 && i < 48)
		finger_name = "little";
	    else if (i >= 48)
		finger_name = "thumb";

	    contacts[finger_name] |= true;
	}
    }

    return true;
}

bool LocalizerModule::getChainJointsState(const std::string &arm_name,
                                          yarp::sig::Vector &arm_angles,
                                          yarp::sig::Vector &torso_angles,
                                          yarp::sig::Vector &arm_ang_rates,
                                          yarp::sig::Vector &torso_ang_rates)
{
    // choose between right and left arm
    yarp::dev::IEncoders *enc;
    if (arm_name == "right")
        enc = ienc_right_arm;
    else
        enc = ienc_left_arm;

    // get the angular readings
    arm_angles.resize(16);
    torso_angles.resize(3);
    bool ok = enc->getEncoders(arm_angles.data());
    if(!ok)
        return false;
    ok = ienc_torso->getEncoders(torso_angles.data());
    if(!ok)
        return false;

    // get the angular rates readings
    arm_ang_rates.resize(16);
    torso_ang_rates.resize(3);
    ok = enc->getEncoderSpeeds(arm_ang_rates.data());
    if (!ok)
        return false;
    ok = ienc_torso->getEncoderSpeeds(torso_ang_rates.data());
    if (!ok)
        return false;

    return true;
}

void LocalizerModule::getHandJointsState(const std::string &hand_name,
                                         const yarp::sig::Vector &arm_angles,
                                         const yarp::sig::Vector &torso_angles,
                                         const yarp::sig::Vector &arm_ang_rates,
                                         const yarp::sig::Vector &torso_ang_rates,
                                         yarp::sig::Vector &hand_angles,
                                         yarp::sig::Vector &hand_ang_rates)
{
    // choose between right and left arm
    iCub::iKin::iCubArm *arm;
    if (hand_name == "right")
        arm = &right_arm_kin;
    else
        arm = &left_arm_kin;

    // fill in the vector of angles
    hand_angles.resize(arm->getDOF());
    hand_angles[0] = torso_angles[2];
    hand_angles[1] = torso_angles[1];
    hand_angles[2] = torso_angles[0];
    hand_angles.setSubvector(3, arm_angles.subVector(0, 6));

    // fill in the vector of angular rates
    hand_ang_rates.resize(arm->getDOF());
    hand_ang_rates[0] = torso_ang_rates[2];
    hand_ang_rates[1] = torso_ang_rates[1];
    hand_ang_rates[2] = torso_ang_rates[0];
    hand_ang_rates.setSubvector(3, arm_ang_rates.subVector(0, 6));
}

void LocalizerModule::getHandTwist(const std::string &hand_name,
                                   const yarp::sig::Vector hand_angles,
                                   const yarp::sig::Vector hand_ang_rates,
                                   yarp::sig::Vector &lin_velocity,
                                   yarp::sig::Vector &ang_velocity)
{
    // choose between right and left hand
    iCub::iKin::iCubArm *arm;
    if (hand_name == "right")
        arm = &right_arm_kin;
    else
        arm = &left_arm_kin;

    // get the geometric jacobian
    yarp::sig::Matrix jac = arm->GeoJacobian();

    // evaluate the twist of the hand
    yarp::sig::Vector twist(6, 0.0);
    twist = jac * (M_PI / 180 * hand_ang_rates);

    // store linear velocity
    lin_velocity = twist.subVector(0, 2);

    // store angular velocity
    ang_velocity = twist.subVector(3, 5);
}

void LocalizerModule::updateFingerConfiguration(const std::string &hand_name,
                                                const std::string &finger_name,
                                                const yarp::sig::Vector &finger_angles)
{
    // get the finger
    iCub::iKin::iCubFinger &finger = fingers_kin[hand_name + "_" + finger_name];

    // update the finger chain
    finger.setAng((M_PI/180) * finger_angles);
}

void LocalizerModule::getFingerJointsState(const std::string &hand_name,
                                           const std::string &finger_name,
                                           const yarp::sig::Vector &arm_angles,
                                           const yarp::sig::Vector &arm_ang_rates,
                                           yarp::sig::Vector &finger_angles,
                                           yarp::sig::Vector &finger_ang_rates)
{
    // get the finger
    iCub::iKin::iCubFinger &finger = fingers_kin[hand_name + "_" + finger_name];

    // get the finger angles
    if (finger_name == "ring")
    {
        finger_angles.resize(4);
        finger_angles[0] = arm_angles[7] / 3.0;
        finger_angles[1] = arm_angles[15] / 3.0;
        finger_angles[3] = finger_angles[2] = finger_angles[1];
    }
    else
        finger.getChainJoints(arm_angles, finger_angles);

    // extract finger angular rates
    if (finger_name == "thumb")
        finger_ang_rates = arm_ang_rates.subVector(8, 10);
    else if (finger_name == "index")
        finger_ang_rates = arm_ang_rates.subVector(11, 12);
    else if (finger_name == "middle")
        finger_ang_rates = arm_ang_rates.subVector(13, 14);
    else if (finger_name == "ring")
        finger_ang_rates = arm_ang_rates.subVector(15, 15);
}

void LocalizerModule::getFingerRelativeVelocity(const std::string &finger_name,
                                                const std::string &hand_name,
                                                const yarp::sig::Vector &finger_angles,
                                                const yarp::sig::Vector &finger_ang_rates,
                                                yarp::sig::Vector &velocity)
{
    // extract joints_speeds
    yarp::sig::Vector speeds = finger_ang_rates;
    if (finger_name == "thumb")
    {
        // up to now only thumb opposition is considered
        speeds = finger_ang_rates.subVector(0, 0);
    }

    // get the finger
    iCub::iKin::iCubFinger &finger = fingers_kin[hand_name + "_" + finger_name];

    // jacobian for linear velocity part
    yarp::sig::Matrix j_lin;

    // get the jacobian
    yarp::sig::Matrix jacobian = finger.GeoJacobian();

    // neglect abduction if index or ring
    if (finger_name == "index" || finger_name == "ring")
        jacobian.removeCols(0, 1);
    // retain only opposition if thumb
    else if (finger_name == "thumb")
        jacobian.removeCols(1, 3);

    // extract linear velocity part
    if (finger_name == "thumb")
        j_lin = jacobian.submatrix(0, 2, 0, 0);
    else
        j_lin = jacobian.submatrix(0, 2, 0, 2);

    // take into account coupling
    yarp::sig::Matrix coupling(3, speeds.size());
    coupling = 0;
    if (finger_name == "index" || finger_name == "middle")
    {
        coupling[0][0] = 1;   // proximal joint velocity = velocity of first DoF
        coupling[1][1] = 0.5; // first distal joint velocity = half velocity of second DoF
        coupling[2][1] = 0.5; // second distal joint velocity = half velocity of second DoF
    }
    else if (finger_name == "ring")
    {
        // within ring only one DoF is available
        coupling = 1.0 / 3.0;
    }
    else if (finger_name == "thumb")
    {
        // only thumb opposition is considered
        coupling.resize(1, 1);
        coupling = 1.0;
    }

    // evaluate relative velocity
    velocity = j_lin * coupling * (M_PI / 180 * speeds);
}

void LocalizerModule::getFingerPosition(const std::string &finger_name,
                                        const std::string &hand_name,
                                        const yarp::sig::Vector &hand_pos,
                                        const yarp::sig::Matrix &hand_att,
                                        yarp::sig::Vector &position)
{
    // get the finger
    iCub::iKin::iCubFinger &finger = fingers_kin[hand_name + "_" + finger_name];

    // get the current position of the finger
    // with respect to the center of the hand
    yarp::sig::Vector finger_pos = finger.EndEffPosition();

    // express it in the robot root frame
    finger_pos = hand_att * finger_pos;

    // find the position of the finger
    // with respect to the robot root frame
    position = hand_pos + finger_pos;
}

void LocalizerModule::getFingerVelocity(const std::string &finger_name,
                                        const std::string &hand_name,
                                        const yarp::sig::Vector &hand_lin_vel,
                                        const yarp::sig::Vector &hand_ang_vel,
                                        const yarp::sig::Vector &finger_angles,
                                        const yarp::sig::Vector &finger_ang_rates,
                                        const yarp::sig::Matrix &hand_2_robot,
                                        yarp::sig::Vector &velocity)
{
    // get the finger
    iCub::iKin::iCubFinger &finger = fingers_kin[hand_name + "_" + finger_name];

    // get the current position of the finger
    // with respect to the center of the hand
    yarp::sig::Vector finger_pose = finger.EndEffPosition();

    // express it in the robot root frame
    finger_pose = hand_2_robot * finger_pose;

    // evaluate the velocity of the finger due to
    // arm joints
    yarp::sig::Vector base_velocity(3, 0.0);
    base_velocity = hand_lin_vel + yarp::math::cross(hand_ang_vel, finger_pose);

    // evaluate the velocity of the finger due to
    // finger joints
    yarp::sig::Vector relative_velocity(3, 0.0);
    getFingerRelativeVelocity(finger_name, hand_name,
                              finger_angles, finger_ang_rates,
                              relative_velocity);

    // express it in the robot root frame
    relative_velocity = hand_2_robot * relative_velocity;

    // compose the total velocity
    velocity = base_velocity + relative_velocity;
}

void LocalizerModule::getFingersData(const std::string &hand_name,
                                     std::unordered_map<std::string, yarp::sig::Vector> &angles,
                                     std::unordered_map<std::string, yarp::sig::Vector> &positions,
                                     std::unordered_map<std::string, yarp::sig::Vector> &lin_vels)
{
    // choose between right and left hand
    iCub::iKin::iCubArm *arm;
    if (hand_name == "right")
        arm = &right_arm_kin;
    else
        arm = &left_arm_kin;

    // get arm and torso joints state
    yarp::sig::Vector arm_angles;
    yarp::sig::Vector torso_angles;
    yarp::sig::Vector arm_ang_rates;
    yarp::sig::Vector torso_ang_rates;
    getChainJointsState(hand_name, arm_angles, torso_angles,
                        arm_ang_rates, torso_ang_rates);

    // get hand joints state
    yarp::sig::Vector hand_angles;
    yarp::sig::Vector hand_ang_rates;
    getHandJointsState(hand_name, arm_angles, torso_angles,
                       arm_ang_rates, torso_ang_rates,
                       hand_angles, hand_ang_rates);

    // update the arm chain
    // (iKin uses radians)
    arm->setAng((M_PI/180) * hand_angles);

    // get hand pose
    yarp::sig::Matrix hand_pose = arm->getH();
    yarp::sig::Vector hand_pos = hand_pose.getCol(3).subVector(0, 2);
    yarp::sig::Matrix hand_att = hand_pose.submatrix(0, 2, 0, 2);

    // get hand twist
    yarp::sig::Vector hand_lin_vel;
    yarp::sig::Vector hand_ang_vel;
    getHandTwist(hand_name, hand_angles, hand_ang_rates,
                 hand_lin_vel, hand_ang_vel);

    // process all the fingers
    for (std::string &finger_name : fingers_names)
    {
        // get finger joints state
        yarp::sig::Vector finger_angles;
        yarp::sig::Vector finger_ang_rates;
        getFingerJointsState(hand_name, finger_name,
                             arm_angles, arm_ang_rates,
                             finger_angles, finger_ang_rates);

        // store angles
        angles[finger_name] = finger_angles;

        // update finger chain
        updateFingerConfiguration(hand_name,
                                  finger_name,
                                  finger_angles);

        // get position
        getFingerPosition(finger_name, hand_name,
                          hand_pos, hand_att,
                          positions[finger_name]);

        // get the total finger velocity
        yarp::sig::Vector velocity;
        getFingerVelocity(finger_name, hand_name,
                          hand_lin_vel, hand_ang_vel,
                          finger_angles,
                          finger_ang_rates,
                          hand_att,
                          velocity);

        // store velocity
        lin_vels[finger_name] = velocity;
    }
}

void LocalizerModule::processCommand(const yarp::sig::FilterCommand &filter_cmd)
{
    // extract command and filtering type
    int cmd = filter_cmd.command();
    int type = filter_cmd.tag();

    if (cmd == VOCAB2('O','N'))
    {
        filtering_enabled = true;
        if (type == VOCAB3('V', 'I', 'S'))
            filtering_type = FilteringType::visual;
        else if (type == VOCAB4('T','A','C','R'))
        {
            filtering_type = FilteringType::tactile;
            tac_filt_hand_name = "right";
        }
        else if (type == VOCAB4('T','A','C','L'))
        {
            filtering_type = FilteringType::tactile;
            tac_filt_hand_name = "left";
        }
    }
    else if (cmd == VOCAB3('O','F','F'))
    {
        stopFiltering();
    }
    else if (cmd == VOCAB3('R','E','S'))
    {
        // first stop filtering
        stopFiltering();

        // then reset filter
        upf.init();
    }
}

void LocalizerModule::performFiltering()
{
    if (!filtering_enabled)
        return;

    // store initial time
    t_i = yarp::os::Time::now();

    // storage for time stamp
    double time_stamp;

    if (filtering_type == FilteringType::visual)
    {
        // check if a point cloud is available
        PointCloud *new_pc = port_pc.read(false);
        if (new_pc == NULL)
        {
            // nothing to do here
            return;
        }

        // transform point cloud
        std::vector<yarp::sig::Vector> pc;
        transformPointCloud(*new_pc,
                            pc);

        // set noise covariances
        upf.setQ(Q_vision);
        upf.setR(R_vision);

        // set alpha parameter
        upf.setAlpha(1.0);

        // process cloud in chuncks of 10 points
        // TODO: take n_points from config
        int n_points = 10;
        for (size_t i=0; i+n_points <= pc.size(); i += n_points)
        {
            // since multiple chuncks are processed
            // the initial time is reset from the second chunck on
            if (i != 0)
            {
                t_i = yarp::os::Time::now();
            }

            // prepare measure
            std::vector<yarp::sig::Vector> measure;
            for (size_t k=0; k<n_points; k++)
                measure.push_back(pc[i+k]);

            // set measure
            upf.setNewMeasure(measure);

            // set zero input (visual localization is static)
            yarp::sig::Vector input(3, 0.0);
            upf.setNewInput(input);

            // step and estimate
            // using time of simulated environment
            // in case env variable YARP_CLOCK is set
            upf.step(time_stamp);
            last_estimate = upf.getEstimate();

            // evaluate final time and execution time
            t_f = yarp::os::Time::now();
            exec_time = t_f - t_i;

            storage_on_mutex.lock();

            // store data if required
            if (storage_on)
                storeData(FilteringType::visual,
                          last_ground_truth,
                          last_estimate,
                          measure,
                          input,
                          time_stamp,
                          exec_time);

            storage_on_mutex.unlock();

            // after a visual filtering step
            // the filter disables automatically
            // TODO: make this optional from configuration
            //       or from FilterCommand
            stopFiltering();

            // a new estimate is now available
            estimate_available = true;
        }
    }
    else if (filtering_type == FilteringType::tactile)
    {
        // check if contacts are available
        std::vector<yarp::sig::Vector> points;
        std::unordered_map<std::string, bool> fingers_contacts;
        if (is_simulation)
        {
            // Gazebo provides contact points
            if (!getContactPointsSim(tac_filt_hand_name, points))
                return;
        }
        else
        {
            // real robot provides binarized information on contacts
            if (!getContacts(tac_filt_hand_name, fingers_contacts))
                return;
        }

        // set noise covariances
        upf.setQ(Q_tactile);
        upf.setR(R_tactile);

        // set alpha parameter
        upf.setAlpha(0.3);

        // get data related to fingers
        yarp::sig::Vector input;
        std::unordered_map<std::string, yarp::sig::Vector> fingers_angles;
        std::unordered_map<std::string, yarp::sig::Vector> fingers_pos;
        std::unordered_map<std::string, yarp::sig::Vector> fingers_vels;
        getFingersData(tac_filt_hand_name, fingers_angles, fingers_pos, fingers_vels);

        if (!is_simulation)
        {
            // extract contact points from forward kinematics
            getContactPoints(fingers_contacts, fingers_pos, points);
        }

        // set input
        input = fingers_vels["middle"];
        input[2] = 0;
        upf.setNewInput(input);

        if (is_first_step)
        {
            upf.resetTime();
            is_first_step = false;
        }

        if (points.size() <= 1)
            // skip step in case of too few measurements
            upf.skipStep(time_stamp);
        else
        {
            // do normal filtering step
            upf.setNewMeasure(points);
            upf.step(time_stamp);
            last_estimate = upf.getEstimate();
        }

        // evaluate final time and execution time
        t_f = yarp::os::Time::now();
        exec_time = t_f - t_i;

        storage_on_mutex.lock();

        // store data if required
        if (storage_on)
            storeDataTactile(last_ground_truth,
                             last_estimate,
                             points,
                             input,
                             fingers_angles,
                             fingers_pos,
                             fingers_vels,
                             time_stamp,
                             exec_time);

        storage_on_mutex.unlock();

        // a new estimate is now available
        estimate_available = true;
    }
}

void LocalizerModule::stopFiltering()
{
    filtering_enabled = false;

    // reset flag for the next activation
    is_first_step = true;
}

void LocalizerModule::publishEstimate()
{
    // convert the estimated pose to a homogeneous transformation matrix
    yarp::math::FrameTransform pose;
    pose.transFromVec(last_estimate[0],
                      last_estimate[1],
                      last_estimate[2]);

    // estimate is saved as x-y-z-Y-P-R
    pose.rotFromRPY(last_estimate[5],
                    last_estimate[4],
                    last_estimate[3]);

    // Set a new transform
    tf_client->setTransform(est_target_frame_name,
                           est_source_frame_name,
                           pose.toMatrix());
}

bool LocalizerModule::retrieveGroundTruth(yarp::sig::Vector &pose)
{
    // Get the pose of the root frame of the robot
    // TODO: get source and target from configuration file
    yarp::sig::Matrix inertialToRobot(4,4);
    std::string source = "/iCub/frame";
    std::string target = "/box_alt/frame";

    // Get the transform
    if (!tf_client->getTransform(target, source, inertialToRobot))
        return false;

    pose.resize(6);

    // Extract position and ZXY euler angles
    yarp::math::FrameTransform frame;
    frame.fromMatrix(inertialToRobot);

    yarp::math::FrameTransform::Translation_t &pos = frame.translation;
    pose[0] = pos.tX;
    pose[1] = pos.tY;
    pose[2] = pos.tZ;

    yarp::sig::Vector angles = frame.getRPYRot();
    pose[3] = angles[2];
    pose[4] = angles[1];
    pose[5] = angles[0];

    return true;
}

void LocalizerModule::resetStorage()
{
    // reset internal storage
    storage.clear();
}

Data& LocalizerModule::storeData(const FilteringType &data_type,
                                 const yarp::sig::Vector &ground_truth,
                                 const yarp::sig::Vector &estimate,
                                 const std::vector<yarp::sig::Vector> &meas,
                                 const yarp::sig::Vector &input,
                                 const double &time_stamp,
                                 const double &exec_time)
{
    Data d;

    // populate
    d.data_type = data_type;
    d.ground_truth = ground_truth;
    d.estimate = estimate;
    d.meas = meas;
    d.input = input;
    d.time_stamp = time_stamp;
    d.exec_time = exec_time;

    // add to storage
    storage.push_back(d);

    // return reference to last element
    return storage.back();
}

void LocalizerModule::storeDataTactile(const yarp::sig::Vector &ground_truth,
                                       const yarp::sig::Vector &estimate,
                                       const std::vector<yarp::sig::Vector> &meas,
                                       const yarp::sig::Vector &input,
                                       std::unordered_map<std::string, yarp::sig::Vector> fingers_joints,
                                       std::unordered_map<std::string, yarp::sig::Vector> fingers_pos,
                                       std::unordered_map<std::string, yarp::sig::Vector> fingers_vels,
                                       const double &time_stamp,
                                       const double &exec_time)
{
    // store common data
    Data &d = storeData(FilteringType::tactile, ground_truth,
                        estimate, meas, input, time_stamp, exec_time);

    // add additional fields
    d.fingers_joints = fingers_joints;
    d.fingers_pos = fingers_pos;
    d.fingers_vels = fingers_vels;
}

bool LocalizerModule::saveMesh(const yarp::sig::Vector &pose,
                               const std::string &file_name)
{
    // obtain the mesh from the filter
    // (the filter contains the model of the object)
    Polyhedron p;
    upf.transformObject(pose, p);

    // save the model
    // overwrite in case it already exists
    std::ofstream fout(file_name.c_str(), std::ios::trunc);
    if(fout.is_open())
        fout << p;
    else
    {
        fout.close();

        yError() << "LocalizerModule: unable to save mesh file to"
                 << file_name;
        return false;
    }

    fout.close();

    return true;
}

bool LocalizerModule::saveMeas(const std::vector<yarp::sig::Vector> &meas,
                               const std::string &file_name)
{
    // save the measures
    // overwrite if it already exists
    std::ofstream fout(file_name.c_str());
    if(fout.is_open())
    {
        // print the OFF header
        fout << "OFF" << std::endl;
        // this is a vertices only .OFF
        fout << meas.size() << " 0 0" << std::endl;

        // save all the measurements
        for (const yarp::sig::Vector &m : meas)
        {
            fout << m[0] << " "
                 << m[1] << " "
                 << m[2] << " "
                 << std::endl;
        }
    }
    else
    {
        fout.close();

        yError() << "LocalizerModule: problem opening meas output file"
                 << file_name;
        return false;
    }

    fout.close();

    return true;
}

bool LocalizerModule::saveFingersJoints(const std::unordered_map<std::string, yarp::sig::Vector> &angles,
                                        const std::string &file_name)
{
    // save the angles
    // overwrite if it already exists
    std::ofstream fout(file_name.c_str(), std::ios::trunc);
    if(fout.is_open())
    {

        // copy data since yarp::sig::Vector::setSubvector does not accept const data
        std::unordered_map<std::string, yarp::sig::Vector> data = angles;

        // print the CSV header
        fout << "finger_id;"
        // no more than four joints are expected
             << "q_0;" << "q_1;" << "q_2;" << "q_3;"
             << std::endl;

        for (std::string &finger_name : fingers_names)
        {
            // finger id
            if (finger_name == "thumb")
                fout << 0;
            else if (finger_name == "index")
                fout << 1;
            else if (finger_name == "middle")
                fout << 2;
            else if (finger_name == "ring")
                fout << 3;
            else
                return false;
            fout << ";";

            // if a joint is not present its value is set to 0
            yarp::sig::Vector values(4, 0.0);

            // copy available joints
            values.setSubvector(0, data[finger_name]);

            // write to file
            fout << values[0] << ";"
                 << values[1] << ";"
                 << values[2] << ";"
                 << values[3] << ";"
                 << std::endl;
        }
    }
    else
    {
        fout.close();

        yError() << "LocalizerModule: problem opening joints angles output file"
                 << file_name;
        return false;
    }

    return true;
}

bool LocalizerModule::saveFingersPositions(const std::unordered_map<std::string, yarp::sig::Vector> &positions,
                                           const std::string &file_name)
{
    // save the positions
    // overwrite if it already exists
    std::ofstream fout(file_name.c_str(), std::ios::trunc);
    if(fout.is_open())
    {
        // copy data since yarp::sig::Vector::setSubvector does not accept const data
        std::unordered_map<std::string, yarp::sig::Vector> data = positions;

        // print the CSV header
        fout << "finger_id;"
             << "pos_x;" << "pos_y;" << "pos_z;"
             << std::endl;

        for (std::string &finger_name : fingers_names)
        {
            // finger id
            if (finger_name == "thumb")
                fout << 0;
            else if (finger_name == "index")
                fout << 1;
            else if (finger_name == "middle")
                fout << 2;
            else if (finger_name == "ring")
                fout << 3;
            else
                return false;
            fout << ";";

            // get velocity
            yarp::sig::Vector &p = data[finger_name];

            // write to file
            fout << p[0] << ";"
                 << p[1] << ";"
                 << p[2] << ";"
                 << std::endl;
        }
    }
    else
    {
        fout.close();

        yError() << "LocalizerModule: problem opening joints angles output file"
                 << file_name;
        return false;
    }

    return true;
}

bool LocalizerModule::saveFingersVelocities(const std::unordered_map<std::string, yarp::sig::Vector> &velocities,
                                            const std::string &file_name)
{
    // save the angles
    // overwrite if it already exists
    std::ofstream fout(file_name.c_str(), std::ios::trunc);
    if(fout.is_open())
    {
        // copy data since yarp::sig::Vector::setSubvector does not accept const data
        std::unordered_map<std::string, yarp::sig::Vector> data = velocities;

        // print the CSV header
        fout << "finger_id;"
             << "vel_x;" << "vel_y;" << "vel_z;"
             << std::endl;

        for (std::string &finger_name : fingers_names)
        {
            // finger id
            if (finger_name == "thumb")
                fout << 0;
            else if (finger_name == "index")
                fout << 1;
            else if (finger_name == "middle")
                fout << 2;
            else if (finger_name == "ring")
                fout << 3;
            else
                return false;
            fout << ";";

            // get velocity
            yarp::sig::Vector &v = data[finger_name];

            // write to file
            fout << v[0] << ";"
                 << v[1] << ";"
                 << v[2] << ";"
                 << std::endl;
        }
    }
    else
    {
        fout.close();

        yError() << "LocalizerModule: problem opening joints angles output file"
                 << file_name;
        return false;
    }

    return true;
}

bool LocalizerModule::saveData(const std::vector<Data> &data)
{
    // compose file name for report file
    std::string report_path = output_path + "report.csv";

    // save the data
    // overwrite in case it already exists
    std::ofstream fout(report_path.c_str(), std::ios::trunc);
    if(fout.is_open())
    {
        // print the CSV header
        fout << "step;" << "type;"
             << "x_real;" << "y_real;" << "z_real;"
             << "phi_real;" << "theta_real;" << "psi_real;"
             << "x_sol;"   << "y_sol;"     << "z_sol;"
             << "phi_sol;" << "theta_sol;" << "psi_sol;"
             << "input_x;"   << "input_y;" << "input_z;"
             << "time_stamp;"
             << "exec_time;"
             << std::endl;

        // save data for each step
        int step_index = 0;
        for (const Data& d : data)
        {
            // index
            fout << step_index << ";";
            // type
            if (d.data_type == FilteringType::visual)
                fout << 0 << ";";
            else if (d.data_type == FilteringType::tactile)
                fout << 1 << ";";
            // ground truth
            for(size_t j=0; j<6; j++)
                fout << d.ground_truth[j] << ";";
            // estimate
            for(size_t j=0; j<6; j++)
                fout << d.estimate[j] << ";";
            // input
            for(size_t j=0; j<3; j++)
                fout << d.input[j] << ";";
            // time stamp
            fout << d.time_stamp << ";";
            // execution time
            fout << d.exec_time << ";";

            fout << std::endl;

            // save measurements separately since
            // their numbers change from step to step
            std::string meas_path = output_path + "meas_step_"
                + std::to_string(step_index) + ".off";
            if (!saveMeas(d.meas, meas_path))
            {
                // error message is provided by saveMeas()
                fout.close();
                return false;
            }

            // save mesh of ground truth pose
            std::string gt_mesh_path = output_path + "gt_mesh_step_"
                + std::to_string(step_index) + ".off";
            if (!saveMesh(d.ground_truth, gt_mesh_path))
            {
                // error message is provided by saveMesh()
                fout.close();
                return false;
            }

            // save mesh of estimated pose
            std::string est_mesh_path = output_path + "est_mesh_step_"
                + std::to_string(step_index) +  ".off";
            if (!saveMesh(d.estimate, est_mesh_path))
            {
                // error message is provided by saveMesh()
                fout.close();
                return false;
            }

            if (d.data_type == FilteringType::tactile)
            {
                // save fingers joints angles
                std::string angles_path = output_path + "fingers_joints_"
                    + std::to_string(step_index) + ".csv";
                if (!saveFingersJoints(d.fingers_joints, angles_path))
                {
                    fout.close();
                    return false;
                }

                std::string positions_path = output_path + "fingers_positions_"
                    + std::to_string(step_index) + ".csv";
                if (!saveFingersPositions(d.fingers_pos, positions_path))
                {
                    fout.close();
                    return false;
                }

                // save fingers velocities
                std::string velocities_path = output_path + "fingers_velocities_"
                    + std::to_string(step_index) + ".csv";
                if (!saveFingersVelocities(d.fingers_vels, velocities_path))
                {
                    fout.close();
                    return false;
                }
            }

            step_index++;
        }
    }
    else
    {
        fout.close();

        yError() << "LocalizeModule: problem opening output report file"
                 << report_path;
        return false;
    }

    fout.close();

    return true;
}

bool LocalizerModule::respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply)
{
    std::string cmd = command.get(0).asString();
    if (cmd == "help")
    {
        reply.addVocab(yarp::os::Vocab::encode("many"));
        reply.addString("Available commands:");
        reply.addString("- storage-on");
        reply.addString("- storage-off");
        reply.addString("- storage-save");
        reply.addString("- reset");
        reply.addString("- quit");
    }
    else if (cmd == "storage-on")
    {
        // change flag
        storage_on_mutex.lock();
        storage_on = true;
        storage_on_mutex.unlock();

        resetStorage();

        reply.addString("Storage enabled succesfully.");
    }
    else if (cmd == "storage-off")
    {
        // change flag
        storage_on_mutex.lock();
        storage_on = false;
        storage_on_mutex.unlock();

        reply.addString("Storage disabled succesfully.");
    }
    else if (cmd == "storage-save")
    {
        bool ok;

        ok = saveData(storage);
        if (ok)
            reply.addString("Storage saved to file succesfully.");
        else
            reply.addString("Storage save failed.");
    }
    else if (cmd == "reset")
    {
        // reset the filter
        upf.init();

        reply.addString("Filter reset successful.");
    }
    else
        // the father class already handles the "quit" command
        return RFModule::respond(command,reply);

    return true;
}

bool LocalizerModule::configure(yarp::os::ResourceFinder &rf)
{
    // load parameters from the configuration file
    // using group 'upf-module'
    yarp::os::ResourceFinder rf_module;
    rf_module = rf.findNestedResourceFinder("upf-module");
    if (!loadParameters(rf_module))
    {
        yError() << "LocalizerModule::Configure error:"
                 << "failure in loading parameters from configuration file";
        return false;
    }

    // open ports
    bool ok_port = port_in.open(input_port_name);

    // stop the configuration if the port open failed
    if (!ok_port)
    {
        yError() << "LocalizerModule::Configure error:"
                 << "failure in opening input port /upf-localizer:i";
        return false;
    }

    ok_port = port_pc.open(port_pc_name);
    if (!ok_port)
    {
        yError() << "LocalizerModule:Configure error:"
                 << "unable to open the point cloud port";
        return false;
    }

    if (is_simulation)
    {
        ok_port = port_contacts_sim.open(port_contacts_name);
        if (!ok_port)
        {
            yError() << "LocalizerModule:Configure error:"
                     << "unable to open the simulated contact points port";
            return false;
        }
    }
    else
    {
        ok_port = port_contacts.open(port_contacts_name);
        if (!ok_port)
        {
            yError() << "LocalizerModule:Configure error:"
                     << "unable to open the contact points port";
            return false;
        }
    }

    // set FIFO policy
    port_in.setStrict();

    // prepare properties for the PolyDriver
    yarp::os::Property propTfClient;
    propTfClient.put("device", "FrameTransformClient");
    propTfClient.put("local", "/upf-localizer/transformClient");
    propTfClient.put("remote", "/transformServer");
    propTfClient.put("period", getPeriod() * 1000);

    // open the driver and obtain a a IFrameTransform view
    tf_client = nullptr;
    bool ok_drv = drv_transform_client.open(propTfClient);
    ok_drv = ok_drv && drv_transform_client.view(tf_client) && tf_client != nullptr;

    // stop configuration if the driver open failed
    // or the view retrieval failed
    // or the IFrameTransform pointer is not valid
    if (!ok_drv)
    {
        yError() << "LocalizerModule::Configure error:"
                 << "failure in opening iFrameTransform interface";
        return false;
    }

    // get the pose of the root frame of the robot
    // required to convert point clouds
    inertial_to_robot.resize(4,4);
    bool ok = false;
    double t0 = yarp::os::SystemClock::nowSystem();
    while (yarp::os::SystemClock::nowSystem() - t0 < 10.0)
    {
        // this might fail if the gazebo pluging
        // publishing the pose is not started yet
        if (tf_client->getTransform(robot_target_frame_name,
                                    robot_source_frame_name,
                                    inertial_to_robot))
        {
            ok = true;
            break;
        }
        yarp::os::SystemClock::delaySystem(1.0);
    }
    if (!ok)
    {
        yError() << "LocalizerModule: unable to get the pose of the root frame of the robot";
        return false;
    }

    // prepare properties for the Encoders
    yarp::os::Property prop_encoders;
    prop_encoders.put("device", "remote_controlboard");
    prop_encoders.put("remote", "/icubSim/right_arm");
    prop_encoders.put("local", "/upf-localizer/encoders/right_arm");
    ok_drv = drv_right_arm.open(prop_encoders);
    if (!ok_drv)
    {
        yError() << "LocalizerModule::configure error:"
                 << "unable to open the Remote Control Board driver for the right arm";
        return false;
    }

    prop_encoders.put("remote", "/icubSim/left_arm");
    prop_encoders.put("local", "/upf-localizer/encoders/left_arm");
    ok_drv = drv_left_arm.open(prop_encoders);
    if (!ok_drv)
    {
        yError() << "LocalizerModule::configure error:"
                 << "unable to open the Remote Control Board driver for the left arm";
        return false;
    }

    prop_encoders.put("remote", "/icubSim/torso");
    prop_encoders.put("local", "/upf-localizer/encoders/torso");
    ok_drv = drv_torso.open(prop_encoders);
    if (!ok_drv)
    {
        yError() << "LocalizerModule::configure error:"
                 << "unable to open the Remote Control Board driver for the torso";
        return false;
    }

    // try to retrieve the views
    bool ok_view = drv_right_arm.view(ienc_right_arm);
    if (!ok_view || ienc_right_arm == 0)
    {
        yError() << "LocalizerModule:configure error:"
                 << "unable to retrieve the Encoders view for the right arm";
        return false;
    }
    ok_view = drv_left_arm.view(ienc_left_arm);
    if (!ok_view || ienc_left_arm == 0)
    {
        yError() << "LocalizerModule:configure error:"
                 << "unable to retrieve the Encoders view for the left arm";
        return false;
    }
    ok_view = drv_torso.view(ienc_torso);
    if (!ok_view || ienc_torso == 0)
    {
        yError() << "LocalizerModule:configure error:"
                 << "unable to retrieve the Encoders view for the torso";
        return false;
    }

    // configure forward kinematics
    right_arm_kin = iCub::iKin::iCubArm("right");
    left_arm_kin = iCub::iKin::iCubArm("left");

    fingers_names = {"thumb", "index", "middle", "ring"};
    for (std::string &finger_name : fingers_names)
    {
        std::string right_finger = "right_" + finger_name;
        std::string left_finger = "left_" + finger_name;
        std::string right_finger_key = right_finger;
        std::string left_finger_key = left_finger;
        if (finger_name == "ring")
        {
            // FIX ME :the forward kinematics of the ring finger is not available
            // using the forward kinematics of the index finger
            right_finger = "right_index";
            left_finger = "left_index";
        }
        fingers_kin[right_finger_key] = iCub::iKin::iCubFinger(right_finger);
        fingers_kin[left_finger_key] = iCub::iKin::iCubFinger(left_finger);
    }

    // Limits update is not required to evaluate the forward kinematics
    // using angles from the encoders
    right_arm_kin.setAllConstraints(false);
    left_arm_kin.setAllConstraints(false);
    // Torso can be moved in general so its links have to be released
    right_arm_kin.releaseLink(0);
    right_arm_kin.releaseLink(1);
    right_arm_kin.releaseLink(2);
    left_arm_kin.releaseLink(0);
    left_arm_kin.releaseLink(1);
    left_arm_kin.releaseLink(2);

    // configure and init the UPF
    // using group 'upf' from the configuration file
    yarp::os::ResourceFinder rf_upf;
    rf_upf = rf.findNestedResourceFinder("upf");
    if(!upf.configure(rf_upf))
        return false;
    upf.init();

    // reset storage
    storage_on = false;
    resetStorage();

    // start rpc server
    rpc_port.open(rpc_port_name);
    attach(rpc_port);

    // reset flags
    estimate_available = false;
    filtering_enabled = false;
    is_first_step = true;

    return true;
}

double LocalizerModule::getPeriod()
{
    // TODO: take from the config
    return period;
}

bool LocalizerModule::updateModule()
{
    // check if the module should stop
    if (isStopping())
        return false;

    // try to get the ground truth
    retrieveGroundTruth(last_ground_truth);

    // try to read a command from the port
    bool should_wait = false;
    cmd = port_in.read(should_wait);

    // process the command
    if (cmd != YARP_NULLPTR)
        processCommand(*cmd);

    // do filtering step
    performFiltering();

    // publish the last estimate available
    if (estimate_available)
        publishEstimate();

    return true;
}

bool LocalizerModule::close()
{
    // close ports
    port_in.close();
    port_pc.close();

    // close drivers
    drv_transform_client.close();
    drv_right_arm.close();
    drv_left_arm.close();
    drv_torso.close();
}

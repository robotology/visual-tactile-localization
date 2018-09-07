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
#include <yarp/math/Rand.h>

// CGAL
#include <CGAL/IO/Polyhedron_iostream.h>

// VTK
#include <vtkRadiusOutlierRemoval.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkPointData.h>

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

bool LocalizerModule::loadListDouble(yarp::os::ResourceFinder &rf,
                                     const std::string &key,
                                     const int &size,
                                     yarp::sig::Vector &list)
{
    if (rf.find(key).isNull())
        return false;

    yarp::os::Bottle* b = rf.find(key).asList();
    if (b == nullptr)
        return false;

    if (b->size() != size)
        return false;

    list.resize(size);
    for (size_t i=0; i<b->size(); i++)
    {
        yarp::os::Value item_v = b->get(i);
        if (item_v.isNull())
            return false;

        if (!item_v.isDouble())
        {
            list.clear();
            return false;
        }

        list[i] = item_v.asDouble();
    }
    return true;
}

bool LocalizerModule::loadListStrings(const yarp::os::ResourceFinder &rf,
                                      const std::string &tag_name,
                                      std::vector<std::string> &list)
{
    bool strings_found = false;
    if (!rf.find(tag_name).isNull())
    {
        yarp::os::Bottle* strings_list = rf.find(tag_name).asList();
        if (strings_list != nullptr)
        {
            if (strings_list->size() == 0)
            {
                list.clear();
                return true;
            }
            for (size_t i=0; i<strings_list->size(); i++)
            {
                yarp::os::Value string_v = strings_list->get(i);
                if (string_v.isString())
                    list.push_back(string_v.asString());
                else
                    break;

                if (i == strings_list->size()-1)
                    strings_found = true;
            }
        }
    }
    if (!strings_found)
        list.clear();

    return strings_found;
}

bool LocalizerModule::loadParameters(yarp::os::ResourceFinder &rf)
{
    robot_name = rf.find("robotName").asString();
    if (rf.find("robotName").isNull())
        robot_name = "icub";
    yInfo() << "Localizer module: robot name" << robot_name;

    is_simulation = rf.find("simulationMode").asBool();
    if (rf.find("simulationMode").isNull())
        is_simulation = false;
    yInfo() << "Localizer module: simulation mode" << is_simulation;

    use_analogs = rf.find("useAnalogs").asBool();
    if (rf.find("useAnalogs").isNull())
        use_analogs = false;
    yInfo() << "Localizer moudle: use analogs" << use_analogs;

    use_analogs_bounds = rf.find("useAnalogsBounds").asBool();
    if (rf.find("useAnalogsBounds").isNull())
        use_analogs_bounds = false;
    yInfo() << "Localizer moudle: use analogs bounds" << use_analogs_bounds;

    est_source_frame_name = rf.find("estimateSourceFrame").asString();
    if (rf.find("estimateSourceFrame").isNull())
        est_source_frame_name = "/iCub/frame";
    yInfo() << "Localizer module: estimate source frame name is" << est_source_frame_name;

    est_target_frame_name = rf.find("estimateTargetFrame").asString();
    if (rf.find("estimateTargetFrame").isNull())
        est_target_frame_name = "/estimate/frame";
    yInfo() << "Localizer module: estimate target frame name is" << est_target_frame_name;

    aux_est_source_frame_name = rf.find("auxEstimateSourceFrame").asString();
    if (rf.find("auxEstimateSourceFrame").isNull())
        aux_est_source_frame_name = "/iCub/frame";
    yInfo() << "Localizer module: aux estimate source frame name is" << aux_est_source_frame_name;

    aux_est_target_frame_name = rf.find("auxEstimateTargetFrame").asString();
    if (rf.find("auxEstimateTargetFrame").isNull())
        aux_est_target_frame_name = "/estimate/aux/frame";
    yInfo() << "Localizer module: aux estimate target frame name is" << aux_est_target_frame_name;

    robot_source_frame_name = rf.find("robotSourceFrame").asString();
    if (rf.find("robotSourceFrame").isNull())
        robot_source_frame_name = "/inertial";
    yInfo() << "Localizer module: robot source frame name is" << robot_source_frame_name;

    robot_target_frame_name = rf.find("robotTargetFrame").asString();
    if (rf.find("robotTargetFrame").isNull())
        robot_target_frame_name = "/iCub/frame";
    yInfo() << "Localizer module: robot target frame name is" << robot_target_frame_name;

    gtruth_source_frame_name = rf.find("groundTruthSourceFrame").asString();
    if (rf.find("estimateSourceFrame").isNull())
        gtruth_source_frame_name = "/iCub/frame";
    yInfo() << "Localizer module: ground truth source frame name is" << gtruth_source_frame_name;

    gtruth_target_frame_name = rf.find("groundTruthTargetFrame").asString();
    if (rf.find("estimateTargetFrame").isNull())
        gtruth_target_frame_name = "/ground_truth/frame";
    yInfo() << "Localizer module: ground truth target frame name is" << gtruth_target_frame_name;

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

    // port_filtered_pc_name = rf.find("filteredPointCloudInputPort").asString();
    // if (rf.find("filteredPointCloudInputPort").isNull())
    //     port_filtered_pc_name = "/upf-localizer/filtered_pc:i";
    // yInfo() << "Localizer module: filtered point cloud input port name is" << port_filtered_pc_name;

    port_contacts_name = rf.find("contactsInputPort").asString();
    if (rf.find("contactsInputPort").isNull())
        port_contacts_name = "/upf-localizer/contacts:i";
    yInfo() << "Localizer module: contact points input port name is" << port_contacts_name;

    if (!rf.check("outputPath"))
    {
        yError() << "Localizer module: output path not provided!";
        return false;
    }
    output_path = rf.find("outputPath").asString();
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

    visual_chunk_size = rf.find("pointCloudVisualChunkSize").asInt();
    if (rf.find("pointCloudVisualChunkSize").isNull())
        visual_chunk_size = 10;

    use_pc_subsampling = rf.find("pointCloudSubsample").asBool();
    if (rf.find("pointCloudSubsample").isNull())
        use_pc_subsampling = true;

    subsample_n_points = rf.find("pointCloudSubsampleNumPoints").asInt();
    if (rf.find("pointCloudSubsampleNumPoints").isNull())
        subsample_n_points = 30;

    use_pc_shuffle = rf.find("pointCloudShuffle").asBool();
    if (rf.find("pointCloudShuffle").isNull())
        use_pc_shuffle = true;

    shuffle_resize_factor = rf.find("pointCloudShuffleResFactor").asDouble();
    if (rf.find("pointCloudShuffleResFactor").isNull())
        shuffle_resize_factor = 0.9;

    use_pc_outlier_rem = rf.find("pointCloudOutlierRemoval").asBool();
    if (rf.find("pointCloudOutlierRemoval").isNull())
        use_pc_outlier_rem = false;

    outlier_rem_radius = rf.find("pointCloudOutlierRadius").asDouble();
    if (rf.find("pointCloudOutlierRadius").isNull())
        outlier_rem_radius = 0.01;

    outlier_rem_neigh = rf.find("pointCloudOutlierNeigh").asInt();
    if (rf.find("pointCloudOutlierNeigh").isNull())
        outlier_rem_neigh = 10;

    use_pc_dense_outlier_rem = rf.find("pointCloudDensOutlierRemoval").asBool();
    if (rf.find("pointCloudDensOutlierRemoval").isNull())
        use_pc_dense_outlier_rem = false;

    dense_outlier_rem_thr = rf.find("pointCloudDensOutlierThr").asDouble();
    if (rf.find("pointCloudDensOutlierThr").isNull())
        dense_outlier_rem_thr = 10.0;

    if (!loadListDouble(rf, "springyFingersThresLeft",
                        5, springy_thres_left))
    {
        yError() << "LocalizerModule: unable to load threshold for contact detetion"
                 << "with left springy fingers";
        return false;
    }
    if (!loadListDouble(rf, "springyFingersThresRight",
                        5, springy_thres_right))
    {
        yError() << "LocalizerModule: unable to load threshold for contact detetion"
                 << "with right springy fingers";
        return false;
    }

    use_springy = rf.find("useSpringyFingersDetection").asBool();
    if (rf.find("useSpringyFingersDetection").isNull())
        use_springy = false;

    use_ext_vel_observer = rf.find("useExternalVelocityObserver").asBool();
    if (rf.find("useExternalVelocityObserver").isNull())
        use_ext_vel_observer = false;

    // fingers list for approaching phase
    if(!loadListStrings(rf, "excludedFingers", excluded_fingers))
        excluded_fingers = {"thumb"};

    return true;
}

bool LocalizerModule::getPointCloudSim(std::vector<yarp::sig::Vector> &pc)
{
    // original point cloud
    PointCloudXYZ *new_pc = port_pc.read(false);
    if (new_pc == NULL)
        return false;

    // transform point clouds to vectors of yarp::sig::Vector
    for (size_t i=0; i<new_pc->size(); i++)
    {
        yarp::sig::DataXYZ &p_data = (*new_pc)(i);
        yarp::sig::Vector p_vector(3);
        p_vector[0] = p_data.x;
        p_vector[1] = p_data.y;
        p_vector[2] = p_data.z;
        pc.push_back(p_vector);
    }

    // transform the points taking into account
    // the root link of the robot
    for (size_t i=0; i<pc.size(); i++)
    {
        yarp::sig::Vector point(4, 0.0);
        point.setSubvector(0, pc[i]);
        point[3] = 1;

        // transform the point so that
        // it is relative to the orign of the robot root frame
        // and expressed in the robot root frame
        point = SE3inv(inertial_to_robot) * point;

        pc[i] = point.subVector(0,2);
    }

    return true;
}

void LocalizerModule::subsamplePointCloud(const std::vector<yarp::sig::Vector> &pc_in,
                                          std::vector<yarp::sig::Vector> &pc_out,
                                          const unsigned int &skip_points)
{
    for (size_t i=0; i<pc_in.size(); i++)
    {
        if ((i % skip_points) == 0)
            pc_out.push_back(pc_in[i]);
    }
}

void LocalizerModule::shufflePointCloud(const std::vector<yarp::sig::Vector> &pc_in,
                                        std::vector<yarp::sig::Vector> &pc_out,
                                        const double &resize_factor)
{
    std::set<unsigned int> idx;
    int subsampled_pc_size = pc_in.size();
    while (idx.size() < (size_t)(resize_factor * subsampled_pc_size))
    {
        unsigned int i = (unsigned int)(Rand::scalar(0.0,1.0) * subsampled_pc_size);
        if (idx.find(i) == idx.end())
        {
            pc_out.push_back(pc_in[i]);
            idx.insert(i);
        }
    }
}

void LocalizerModule::removeOutliersFromPointCloud(const std::vector<yarp::sig::Vector> &pc_in,
                                                   std::vector<yarp::sig::Vector> &pc_out,
                                                   const double &radius, const int num_points)
{
    vtkSmartPointer<vtkPoints> vtk_points=vtkSmartPointer<vtkPoints>::New();
    for (size_t i=0; i<pc_in.size(); i++)
    {
        const yarp::sig::Vector &point = pc_in[i];
        vtk_points->InsertNextPoint(point[0], point[1], point[2]);
    }

    vtkSmartPointer<vtkPolyData> vtk_polydata=vtkSmartPointer<vtkPolyData>::New();
    vtk_polydata->SetPoints(vtk_points);

    vtkSmartPointer<vtkRadiusOutlierRemoval> removal=vtkSmartPointer<vtkRadiusOutlierRemoval>::New();
    removal->SetInputData(vtk_polydata);
    removal->SetRadius(radius);
    removal->SetNumberOfNeighbors(num_points);
    removal->Update();

    for (size_t i=0; i<pc_in.size(); i++)
    {
        if (removal->GetPointMap()[i]>=0)
            pc_out.push_back(pc_in[i]);
    }
}

void LocalizerModule::removeDenseOutliersFromPointCloud(const double &threshold,
                                                        const std::vector<yarp::sig::Vector> &pc_in,
                                                        std::vector<yarp::sig::Vector> &pc_out)
{
    yarp::sig::Vector mean(3, 0.0);
    for (size_t i=0; i<pc_in.size(); i++)
        mean += pc_in[i];
    mean /= pc_in.size();

    for (size_t i=0; i<pc_in.size(); i++)
    {
        if (yarp::math::norm(pc_in[i] - mean) < threshold)
            pc_out.push_back(pc_in[i]);
    }
}

bool LocalizerModule::getPointCloud(std::vector<yarp::sig::Vector> &pc)
{
    // original point cloud
    PointCloudXYZ *new_pc = port_pc.read(false);
    if (new_pc == NULL)
        return false;

    // transform point clouds to vectors of yarp::sig::Vector
    for (size_t i=0; i<new_pc->size(); i++)
    {
        yarp::sig::DataXYZ &p_data = (*new_pc)(i);
        yarp::sig::Vector p_vector(3);
        p_vector[0] = p_data.x;
        p_vector[1] = p_data.y;
        p_vector[2] = p_data.z;
        pc.push_back(p_vector);
    }

    return true;
}

void LocalizerModule::setupAnalogBounds()
{
    // taken from real robot
    analog_bounds.resize(16, 2);
    analog_bounds(0, 0) = 235.0;
    analog_bounds(1, 0) = 209.0;
    analog_bounds(2, 0) = 237.0;
    analog_bounds(3, 0) = 245.0;
    analog_bounds(4, 0) = 219.0;
    analog_bounds(5, 0) = 233.0;
    analog_bounds(6, 0) = 245.0;
    analog_bounds(7, 0) = 217.0;
    analog_bounds(8, 0) = 249.0;
    analog_bounds(9, 0) = 235.0;
    analog_bounds(10, 0) = 208.0;
    analog_bounds(11, 0) = 234.0;
    analog_bounds(12, 0) = 250.0;
    analog_bounds(13, 0) = 216.0;
    analog_bounds(14, 0) = 230.0;
    analog_bounds(15, 0) = 0.0;
    analog_bounds(0, 1) = 44.0;
    analog_bounds(1, 1) = 14.0;
    analog_bounds(2, 1) = 10.0;
    analog_bounds(3, 1) = 14.0;
    analog_bounds(4, 1) = 36.0;
    analog_bounds(5, 1) = 0.0;
    analog_bounds(6, 1) = 0.0;
    analog_bounds(7, 1) = 6.0;
    analog_bounds(8, 1) = 21.0;
    analog_bounds(9, 1) = 3.0;
    analog_bounds(10, 1) = 25.0;
    analog_bounds(11, 1) = 0.0;
    analog_bounds(12, 1) = 0.0;
    analog_bounds(13, 1) = 39.0;
    analog_bounds(14, 1) = 19.0;
    analog_bounds(15, 0) = 0.0;
}

void LocalizerModule::getContactPoints(const std::unordered_map<std::string, bool> &fingers_contacts,
                                       const std::unordered_map<std::string, yarp::sig::Vector> &fingers_pos,
                                       std::unordered_map<std::string, yarp::sig::Vector> &contact_points)
{
    // clear vector
    contact_points.clear();

    for (auto it = fingers_contacts.begin(); it != fingers_contacts.end(); it++)
    {
        const std::string &finger_name = it->first;
        const bool &is_contact = it->second;

        if (is_contact)
            contact_points[finger_name] = fingers_pos.at(finger_name);
    }
}

bool LocalizerModule::getContactsSim(const std::string &hand_name,
                                     std::unordered_map<std::string, bool> &contacts,
                                     std::unordered_map<std::string, yarp::sig::Vector> &contact_points)
{
    // check if new data is available
    iCub::skinDynLib::skinContactList *list;
    list = port_contacts_sim.read(false);
    if (list == NULL)
        return false;

    // reset finger_contacts
    contacts["thumb"] = false;
    contacts["index"] = false;
    contacts["middle"] = false;
    contacts["ring"] = false;
    contacts["little"] = false;

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
        int taxel_id = taxels_ids[0];
        // taxels ids for finger tips are between 0 and 59
        if (taxel_id >= 0 && taxel_id < 60)
        {
            // extract finger name
            std::string finger_name;
            if ((taxel_id >=0) && (taxel_id < 12))
                finger_name = "index";
            else if ((taxel_id >= 12) && (taxel_id < 24))
                finger_name = "middle";
            else if ((taxel_id) >= 24 && (taxel_id < 36))
                finger_name = "ring";
            else if ((taxel_id >= 36) && (taxel_id < 48))
                finger_name = "little";
            else if (taxel_id >= 48)
                finger_name = "thumb";

            if (((hand_name == "right") &&
                (skin_contact.getSkinPart() == iCub::skinDynLib::SkinPart::SKIN_RIGHT_HAND)) ||
                ((hand_name == "left") &&
                (skin_contact.getSkinPart() == iCub::skinDynLib::SkinPart::SKIN_LEFT_HAND)))
            {
                contacts[finger_name] |= true;
                contact_points[finger_name] = skin_contact.getGeoCenter();
            }
        }
    }

    // exclude fingers if requested
    if (excluded_fingers.size() != 0)
        for (std::string &name : excluded_fingers)
        {
            contacts.at(name) = false;
            contact_points.erase(name);
        }

    return true;
}

bool LocalizerModule::getContactsSpringy(const std::string &hand_name,
                                         std::unordered_map<std::string, bool> &contacts)
{
    // get the correct hand
    yarp::os::Value springy_output;
    yarp::sig::Vector springy_thres;
    yarp::os::Bottle *list;
    if (hand_name == "right")
    {
        right_springy_fingers.getOutput(springy_output);
        springy_thres = springy_thres_right;
    }
    else if (hand_name == "left")
    {
        left_springy_fingers.getOutput(springy_output);
        springy_thres = springy_thres_left;
    }
    else
        return false;
    list = springy_output.asList();

    // reset finger_contacts
    contacts["thumb"] = false;
    contacts["index"] = false;
    contacts["middle"] = false;
    contacts["ring"] = false;
    contacts["little"] = false;

    //yInfo() << "Springy Contacts";
    for (size_t i=0; i<5; i++)
    {
        // yInfo() << fingers_names.at(i)
        //         << ": " << list->get(i).asDouble();
        if (list->get(i).asDouble() > springy_thres[i])
            contacts[fingers_names.at(i)] = true;
        // yInfo() << "";
    }

    // exclude fingers if requested
    if (excluded_fingers.size() != 0)
        for (std::string &name : excluded_fingers)
            contacts.at(name) = false;

    return true;
}

bool LocalizerModule::getContactsTactile(const std::string &hand_name,
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

    // exclude fingers if requested
    if (excluded_fingers.size() != 0)
        for (std::string &name : excluded_fingers)
            contacts.at(name) = false;

    return true;
}

bool LocalizerModule::getChainJointsState(const std::string &arm_name,
                                          yarp::sig::Vector &arm_angles,
                                          yarp::sig::Vector &torso_angles,
                                          yarp::sig::Vector &arm_ang_rates,
                                          yarp::sig::Vector &torso_ang_rates,
                                          yarp::sig::Vector &fingers_analogs)
{
    // choose between right and left arm
    yarp::dev::IEncoders *enc;
    yarp::dev::IAnalogSensor *analog;
    // yarp::os::BufferedPort<yarp::sig::Vector> *ext_vel_obs_arm;
    if (arm_name == "right")
    {
        enc = ienc_right_arm;
        analog = ianalog_right;
        // ext_vel_obs_arm = &ext_vel_obs_right_arm;
    }
    else
    {
        enc = ienc_left_arm;
        analog = ianalog_left;
        // ext_vel_obs_arm = &ext_vel_obs_left_arm;
    }

    // get the angular readings
    arm_angles.resize(16);
    torso_angles.resize(3);
    bool ok = enc->getEncoders(arm_angles.data());
    if (is_simulation)
    {
        // hack encoders in simulation in order to simulate
        // mismatch between visual and tactile domains
        // arm_angles[4] -= 20;
        // arm_angles[5] += 0;
        // arm_angles[6] += 20;
        //
    }
    if(!ok)
        return false;
    ok = ienc_torso->getEncoders(torso_angles.data());
    if(!ok)
        return false;

    // get the angular rates readings
    arm_ang_rates.resize(16);
    torso_ang_rates.resize(3);
    if (use_ext_vel_observer)
    {
        yarp::sig::Vector angles(19);
        yarp::sig::Vector rates(19);
        angles.setSubvector(0, torso_angles);
        angles.setSubvector(3, arm_angles);

        iCub::ctrl::AWPolyElement el(angles, yarp::os::Time::now());

        rates = joints_vel_estimator->estimate(el);
        torso_ang_rates = rates.subVector(0, 2);
        arm_ang_rates = rates.subVector(3, 18);
    }
    else
    {
        ok = enc->getEncoderSpeeds(arm_ang_rates.data());
        if (!ok)
            return false;
        ok = ienc_torso->getEncoderSpeeds(torso_ang_rates.data());
        if (!ok)
            return false;
    }

    if (use_analogs)
    {
        // get additional encoders for proximal and distal joints of fingers
        ok = analog->read(fingers_analogs);
        if(ok != yarp::dev::IAnalogSensor::AS_OK)
            return false;
    }

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
    iCub::iKin::iCubFingerExt &finger = fingers_kin[hand_name + "_" + finger_name];

    // update the finger chain
    finger.setAng((M_PI/180) * finger_angles);
}

void LocalizerModule::getFingerJointsState(const std::string &hand_name,
                                           const std::string &finger_name,
                                           const yarp::sig::Vector &arm_angles,
                                           const yarp::sig::Vector &arm_ang_rates,
                                           const yarp::sig::Vector &finger_analogs,
                                           yarp::sig::Vector &finger_angles,
                                           yarp::sig::Vector &finger_ang_rates)
{
    // get the finger
    iCub::iKin::iCubFingerExt &finger = fingers_kin[hand_name + "_" + finger_name];

    if (use_analogs)
    {
        if (use_analogs_bounds)
            finger.getChainJoints(arm_angles, finger_analogs, finger_angles, analog_bounds);
        else
            finger.getChainJoints(arm_angles, finger_analogs, finger_angles);
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
    iCub::iKin::iCubFingerExt &finger = fingers_kin[hand_name + "_" + finger_name];

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
    iCub::iKin::iCubFingerExt &finger = fingers_kin[hand_name + "_" + finger_name];

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
    iCub::iKin::iCubFingerExt &finger = fingers_kin[hand_name + "_" + finger_name];

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
    yarp::sig::Vector fingers_analogs;
    getChainJointsState(hand_name, arm_angles, torso_angles,
                        arm_ang_rates, torso_ang_rates,
                        fingers_analogs);

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
                             fingers_analogs,
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

bool LocalizerModule::calibrateHand(const std::string &hand_name)
{
    yarp::dev::IAnalogSensor *analog;
    if (hand_name == "right")
        analog = ianalog_right;
    else if (hand_name == "left")
        analog = ianalog_left;
    else
        return false;

    // read analogs
    // at this point it is supposed that the hand
    // is completely opened
    yarp::sig::Vector fingers_analogs;
    bool ok = analog->read(fingers_analogs);
    if(ok != yarp::dev::IAnalogSensor::AS_OK)
        return false;

    // set the new upper bounds
    analog_bounds.setCol(0, fingers_analogs);

    return true;
}

void LocalizerModule::processCommand(const yarp::sig::FilterCommand &filter_cmd)
{
    mutex.lock();

    // extract command and filtering type
    int cmd = filter_cmd.command();
    int type = filter_cmd.tag();

    if (cmd == yarp::os::createVocab('O','N'))
    {
        filtering_enabled = true;
        if (type == yarp::os::createVocab('V', 'I', 'S'))
        {
            filtering_type = FilteringType::visual;
            // reset flag
            is_vis_tac_mismatch = false;
        }
        else if (type == yarp::os::createVocab('T','A','C','R'))
        {
            filtering_type = FilteringType::tactile;
            hand_name = "right";
        }
        else if (type == yarp::os::createVocab('T','A','C','L'))
        {
            filtering_type = FilteringType::tactile;
            hand_name = "left";
        }
        else if (type == yarp::os::createVocab('V', 'T', 'M', 'R'))
        {
            filtering_type = FilteringType::visuo_tactile_matching;
            hand_name = "right";
            vis_tac_mismatch_counter = 0;

            // initialize auxiliary filter with initial condition
            // from the main filter
            std::deque<ParticleUPF> particles;
            upf0.getParticleSet(particles);
            upf1.setParticleSet(particles);
        }
        else if (type == yarp::os::createVocab('V', 'T', 'M', 'L'))
        {
            filtering_type = FilteringType::visuo_tactile_matching;
            hand_name = "left";
            vis_tac_mismatch_counter = 0;

            // initialize auxiliary filter with initial condition
            // from the main filter
            std::deque<ParticleUPF> particles;
            upf0.getParticleSet(particles);
            upf1.setParticleSet(particles);
        }
    }
    else if (cmd == yarp::os::createVocab('O','F','F'))
    {
        stopFiltering();
    }
    else if (cmd == yarp::os::createVocab('R','E','S'))
    {
        // first stop filtering
        stopFiltering();

        // then reset filters
        initFilters();
    }
    else if (cmd == yarp::os::createVocab('P','R','O','N'))
    {
        contacts_probe_enabled = true;
        if (type == yarp::os::createVocab('R', 'I', 'G', 'H'))
            hand_name = "right";
        else if (type == yarp::os::createVocab('L', 'E', 'F', 'T'))
            hand_name = "left";
    }
    else if (cmd == yarp::os::createVocab('P','R','O','F'))
    {
        contacts_probe_enabled = false;
    }

    mutex.unlock();
}

bool LocalizerModule::getProcessedPointCloud(std::vector<yarp::sig::Vector> &point_cloud,
                                             std::vector<yarp::sig::Vector> &filtered_point_cloud)
{
    // check if a point cloud is available
    if (is_simulation)
    {
        if(!getPointCloudSim(filtered_point_cloud))
        {
            // nothing to do here
            return false;
        }
    }
    else
    {
        if(!getPointCloud(point_cloud))
        {
            // nothing to do here
            return false;
        }

        bool outlier_rem;
        bool dense_outlier_rem;
        bool subsampling;
        bool shuffling;
        mutex.lock();
        outlier_rem = use_pc_outlier_rem;
        dense_outlier_rem = use_pc_dense_outlier_rem;
        subsampling = use_pc_subsampling;
        shuffling = use_pc_shuffle;
        mutex.unlock();

        // remove outliers
        std::vector<yarp::sig::Vector> inliers_pc;
        if (outlier_rem)
        {
            double radius;
            int neigh;
            mutex.lock();
            radius = outlier_rem_radius;
            neigh = outlier_rem_neigh;
            mutex.unlock();

            removeOutliersFromPointCloud(point_cloud, inliers_pc,
                                         radius, neigh);
        }
        else
            inliers_pc = point_cloud;

        // remove dense outliers
        std::vector<yarp::sig::Vector> inliers_pc_alt;
        if (dense_outlier_rem)
        {
            double threshold;
            mutex.lock();
            threshold = dense_outlier_rem_thr;
            mutex.unlock();

            removeDenseOutliersFromPointCloud(threshold, inliers_pc, inliers_pc_alt);
        }
        else
            inliers_pc_alt = inliers_pc;

        // subsample point cloud
        std::vector<yarp::sig::Vector> subsampled_pc;
        if (subsampling)
        {
            int n_points;
            mutex.lock();
            n_points = subsample_n_points;
            mutex.unlock();

            subsamplePointCloud(inliers_pc_alt, subsampled_pc, n_points);
        }
        else
            subsampled_pc = inliers_pc;

        // shuffle point cloud
        if (shuffling)
        {
            double resize_factor;
            mutex.lock();
            resize_factor = shuffle_resize_factor;
            mutex.unlock();

            shufflePointCloud(subsampled_pc, filtered_point_cloud, resize_factor);
        }
        else
            filtered_point_cloud = subsampled_pc;
    }

    return true;
}

void LocalizerModule::performVisualFiltering()
{
    // store initial time
    t_i = yarp::os::Time::now();

    // storage for time stamp
    double time_stamp;

    // point cloud
    std::vector<yarp::sig::Vector> point_cloud;
    std::vector<yarp::sig::Vector> filtered_point_cloud;
    if (!getProcessedPointCloud(point_cloud, filtered_point_cloud))
        return;

    // clear inputs used during tactile localization
    upf0.clearInputs();
    // upf1.clearInputs();
    // upf_vis.clearInputs();
    // upf_pred.clearInputs();

    // set noise covariances
    upf0.setQ(Q_vision);
    // upf1.setQ(Q_vision);
    // upf_vis.setQ(Q_vision);
    // upf_pred.setQ(Q_vision);
    upf0.setR(R_vision);
    // upf1.setR(R_vision);
    // upf_vis.setR(R_vision);
    // upf_pred.setR(R_vision);

    // set alpha parameter
    upf0.setAlpha(1.0);
    // upf1.setAlpha(1.0);
    // upf_vis.setAlpha(1.0);
    // upf_pred.setAlpha(1.0);

    // the variable 'all_meas' contains the measurements
    // due to all the chunks
    // this is saved for logging purposes
    std::vector<yarp::sig::Vector> all_meas;
    for (size_t i=0;
         i+visual_chunk_size <= filtered_point_cloud.size();
         i += visual_chunk_size)
    {
        for (size_t k=0; k<visual_chunk_size; k++)
        {
            all_meas.push_back(filtered_point_cloud[i+k]);
        }
    }

    // perform filtering
    for (size_t i=0;
         i+visual_chunk_size <= filtered_point_cloud.size();
         i += visual_chunk_size)
    {
        // since multiple chuncks are processed
        // the initial time is reset from the second chunck on
        if (i != 0)
        {
            t_i = yarp::os::Time::now();
        }

        // prepare measure
        std::vector<yarp::sig::Vector> measure;
        for (size_t k=0; k<visual_chunk_size; k++)
            measure.push_back(filtered_point_cloud[i+k]);

        // set measure
        upf0.setNewMeasure(measure);
        // upf1.setNewMeasure(measure);
        // upf_vis.setNewMeasure(measure);
        // upf_pred.setNewMeasure(measure);

        // set zero input (visual localization is static)
        yarp::sig::Vector input(3, 0.0);
        upf0.setNewInput(input);
        // upf1.setNewInput(input);
        // upf_vis.setNewInput(input);
        // upf_pred.setNewInput(input);

        // step
        upf0.step(time_stamp);
        // upf1.step(time_stamp);
        // upf_vis.step(time_stamp);
        // upf_pred.step(time_stamp);

        // extract estimate
        last_aux_estimate = last_estimate = upf0.getEstimate();
        // last_aux_estimate = upf1.getEstimate();
        // last_vis_estimate = upf_vis.getEstimate();
        // last_pred_estimate = upf_pred.getEstimate();

        // extract all the particles for logging purposes
        std::vector<yarp::sig::Vector> particles;
        upf0.getParticles(particles);

        // evaluate final time and execution time
        t_f = yarp::os::Time::now();
        exec_time = t_f - t_i;

        storage_on_mutex.lock();

        // store data if required
        if (storage_on)
        {
            if (is_simulation)
                point_cloud = filtered_point_cloud;

            bool is_first_chunk = (i == 0);
            storeDataVisual(FilteringType::visual,
                            is_first_chunk,
                            last_ground_truth,
                            last_estimate,
                            last_aux_estimate,
                            last_vis_estimate,
                            last_pred_estimate,
                            particles,
                            measure,
                            input,
                            all_meas,
                            point_cloud,
                            time_stamp,
                            exec_time);
        }

        storage_on_mutex.unlock();

        // a new estimate is now available
        estimate_available = true;
    }
}

bool LocalizerModule::getContacts(std::unordered_map<std::string, bool> &contacts_tactile,
                                  std::unordered_map<std::string, bool> &contacts_springy,
                                  std::unordered_map<std::string, bool> &contacts_all,
                                  std::unordered_map<std::string, yarp::sig::Vector> &sim_contact_points)
{
    if (is_simulation)
    {
        // Gazebo provides contacts state and contact points
        if(!getContactsSim(hand_name, contacts_tactile, sim_contact_points))
            return false;
    }
    else
    {
        // real robot provides binarized information on contacts
        if (!getContactsTactile(hand_name, contacts_tactile))
            return false;
        if (use_springy)
        {
            // use also springy fingers model to detect contacts
            if (!getContactsSpringy(hand_name, contacts_springy))
                return false;
        }
    }

    // fuse tactile contacts with springy contacts
    if (use_springy)
        for (auto it=contacts_tactile.begin(); it!=contacts_tactile.end(); it++)
        {
            contacts_all[it->first] = contacts_tactile[it->first] ||
                contacts_springy[it->first];
        }
    else
    {
        // all the contacts are those due to tactile perception
        contacts_all = contacts_tactile;

        // clear contacts according to springy fingers
        for (std::string &finger_name : fingers_names)
        {
            contacts_springy[finger_name] = false;
        }
    }

    return true;
}

void LocalizerModule::performTactileFiltering()
{
    // store initial time
    t_i = yarp::os::Time::now();

    // storage for time stamp
    double time_stamp;

    // retrieve contacts
    std::unordered_map<std::string, bool> contacts_tactile;
    std::unordered_map<std::string, bool> contacts_springy;
    std::unordered_map<std::string, bool> contacts_all;
    std::unordered_map<std::string, yarp::sig::Vector> sim_contact_points;

    if (!getContacts(contacts_tactile, contacts_springy,
                     contacts_all, sim_contact_points))
        return;

    // get data related to fingers
    std::unordered_map<std::string, yarp::sig::Vector> fingers_angles;
    std::unordered_map<std::string, yarp::sig::Vector> fingers_pos;
    std::unordered_map<std::string, yarp::sig::Vector> fingers_vels;
    getFingersData(hand_name, fingers_angles, fingers_pos, fingers_vels);

    // extract contact points
    std::unordered_map<std::string, yarp::sig::Vector> contact_points;
    if (is_simulation)
    {
        contact_points = sim_contact_points;
    }
    else
    {
        // extract contact points from forward kinematics
        getContactPoints(contacts_all, fingers_pos, contact_points);
    }

    // copy to a vector of yarp::sig::Vector(s)
    std::vector<yarp::sig::Vector> points;
    for (auto it=contact_points.begin(); it!=contact_points.end(); it++)
    {
        points.push_back(it->second);
    }
    yInfo() << "";
    yInfo() << "No. of contacts detected:" << points.size();

    // set parameters
    upf0.setQ(Q_tactile);
    // upf_pred.setQ(Q_tactile);
    upf0.setR(R_tactile);
    upf0.setAlpha(0.3);
    // upf_pred.setAlpha(0.3);

    // set input
    yarp::sig::Vector input;
    input = fingers_vels["middle"];
    input[2] = 0;
    upf0.setNewInput(input);
    // upf_pred.setNewInput(input);

    if (is_vis_tac_mismatch)
    {
        // if visuo tactile mismatch
        // is available update also the auxiliary filter
        upf1.setQ(Q_tactile);
        upf1.setR(R_tactile);
        upf1.setAlpha(0.3);
        upf1.setNewInput(input);
    }

    if (is_first_step)
    {
        // reset internal time requiresd to
        // evaluate velocity increments
        upf0.resetTime();
        // upf_pred.resetTime();
        if (is_vis_tac_mismatch)
            upf1.resetTime();

        is_first_step = false;
    }

    // filtering
    std::vector<yarp::sig::Vector> corrected_points;
    if (points.size() <= 1)
    {
        // skip step in case of too few measurements
        upf0.skipStep(time_stamp);
        // upf_pred.skipStep(time_stamp);
        if (is_vis_tac_mismatch)
            upf1.skipStep(time_stamp);
    }
    else
    {

        // correct measurements using the visuo tactile mismatch
        if (is_vis_tac_mismatch)
        {
            // update the auxiliary filter
            upf1.setNewMeasure(points);
            upf1.step(time_stamp);
            last_aux_estimate = upf1.getEstimate();

            // correct measurements
            correctMeasurements(last_aux_estimate, vis_tac_mismatch,
                                points, corrected_points);
        }
        else
            corrected_points = points;

        // set measures
        upf0.setNewMeasure(corrected_points);

        // do normal filtering step
        upf0.step(time_stamp);
        bool skip_correction = true;
        // upf_pred.step(time_stamp, skip_correction);
        last_estimate = upf0.getEstimate();
        // last_pred_estimate = upf_pred.getEstimate();
    }

    // extract all the particles for logging purposes
    std::vector<yarp::sig::Vector> particles;
    upf0.getParticles(particles);

    // evaluate final time and execution time
    t_f = yarp::os::Time::now();
    exec_time = t_f - t_i;

    storage_on_mutex.lock();

    // store data if required
    if (storage_on)
        storeDataTactile(last_ground_truth,
                         last_aux_estimate,
                         last_estimate,
                         last_vis_estimate,
                         last_pred_estimate,
                         particles,
                         points,
                         corrected_points,
                         input,
                         fingers_angles,
                         fingers_pos,
                         fingers_vels,
                         contacts_tactile,
                         contacts_springy,
                         time_stamp,
                         exec_time);

    storage_on_mutex.unlock();

    // a new estimate is now available
    estimate_available = true;
}

void LocalizerModule::performVisuoTactileMatching()
{
    // store initial time
    t_i = yarp::os::Time::now();

    // storage for time stamp
    double time_stamp;

    // get data related to fingers
    std::unordered_map<std::string, yarp::sig::Vector> fingers_angles;
    std::unordered_map<std::string, yarp::sig::Vector> fingers_pos;
    std::unordered_map<std::string, yarp::sig::Vector> fingers_vels;
    getFingersData(hand_name, fingers_angles, fingers_pos, fingers_vels);

    std::unordered_map<std::string, bool> contacts_tactile;
    std::unordered_map<std::string, bool> contacts_springy;
    std::unordered_map<std::string, bool> contacts_all;

    // reset contacts
    for (std::string &finger_name : fingers_names)
    {
        contacts_springy[finger_name] = false;
        contacts_tactile[finger_name] = false;
    }

    // retrieve contacts
    std::vector<yarp::sig::Vector> points;
    std::unordered_map<std::string, yarp::sig::Vector> sim_contact_points;
    if (getContacts(contacts_tactile, contacts_springy,
                     contacts_all, sim_contact_points))
    {
        // extract contact points
        std::unordered_map<std::string, yarp::sig::Vector> contact_points;
        // extract contact points from forward kinematics
        getContactPoints(contacts_all, fingers_pos, contact_points);

        // copy to a vector of yarp::sig::Vector(s)
        for (auto it=contact_points.begin(); it!=contact_points.end(); it++)
        {
            points.push_back(it->second);
        }
    }

    //if (points.size() <= 1)
    //{
    // testing, always use fingers positions
        // in case of no contact use the position of the fingers anyway
        points.clear();
        for (auto it=fingers_pos.begin(); it!=fingers_pos.end(); it++)
        {
            if (std::find(excluded_fingers.begin(), excluded_fingers.end(), it->first)
                == excluded_fingers.end())
                points.push_back(it->second);
        }
    //}

    // set parameters
    upf1.setQ(Q_tactile);
    upf1.setR(R_tactile);
    upf1.setAlpha(0.3);

    // set zero input
    yarp::sig::Vector input(3, 0.0);
    upf1.setNewInput(input);

    // step auxiliary filter
    upf1.setNewMeasure(points);
    upf1.step(time_stamp);
    last_aux_estimate = upf1.getEstimate();

    // update the visuo tactile mismatch
    evaluateVisualTactileMismatch(last_estimate,
                                  last_aux_estimate,
                                  vis_tac_mismatch);
    is_vis_tac_mismatch = true;

    // here after data from the upf0 is logged

    // extract all the particles for logging purposes
    std::vector<yarp::sig::Vector> particles;
    upf0.getParticles(particles);

    // evaluate final time and execution time
    t_f = yarp::os::Time::now();
    exec_time = t_f - t_i;

    storage_on_mutex.lock();

    // store data if required
    if (storage_on)
        storeDataTactile(last_ground_truth,
                         last_aux_estimate,
                         last_estimate,
                         last_vis_estimate,
                         last_pred_estimate,
                         particles,
                         points,
                         points,
                         input,
                         fingers_angles,
                         fingers_pos,
                         fingers_vels,
                         contacts_tactile,
                         contacts_springy,
                         time_stamp,
                         exec_time);

    storage_on_mutex.unlock();

    // a new estimate is now available
    estimate_available = true;

    // update the counter
    vis_tac_mismatch_counter++;

    // check for end of the procedure
    if (vis_tac_mismatch_counter > 100)
    {
        stopFiltering();
        yInfo() << "Visual tactile matching completed!";
        yInfo() << vis_tac_mismatch.toString();
    }
}

void LocalizerModule::performFiltering()
{
    if (!filtering_enabled)
        return;

    if (filtering_type == FilteringType::visual)
        performVisualFiltering();
    else if (filtering_type == FilteringType::tactile)
        performTactileFiltering();
    else if (filtering_type == FilteringType::visuo_tactile_matching)
        performVisuoTactileMatching();
}

void LocalizerModule::performContactsProbe()
{
    if (!contacts_probe_enabled)
        return;

    // get data from forward kinematics
    std::unordered_map<std::string, yarp::sig::Vector> fingers_angles;
    std::unordered_map<std::string, yarp::sig::Vector> fingers_pos;
    std::unordered_map<std::string, yarp::sig::Vector> fingers_vels;
    std::unordered_map<std::string, yarp::sig::Vector> fingers_points;
    getFingersData(hand_name, fingers_angles, fingers_pos, fingers_vels);

    // contacts detected using springy fingers
    std::unordered_map<std::string, bool> springy_contacts;
    getContactsSpringy(hand_name, springy_contacts);

    // extract contact points
    getContactPoints(springy_contacts, fingers_pos, fingers_points);

    // print information
    for (auto it = springy_contacts.begin();
         it != springy_contacts.end();
         it++)
    {
        if (it->second)
        {
            yInfo() << "[SPRINGY][" << hand_name << "hand]"
                    << it->first;
            yInfo() << "@"
                    << fingers_points.at(it->first).toString();
        }
    }

    // contacts from tactile sensors of fingertips
    std::unordered_map<std::string, bool> fingers_contacts;
    if (is_simulation)
    {
        // Gazebo provides contacts state and contact points
        if (!getContactsSim(hand_name, fingers_contacts, fingers_points))
            return;
    }
    else
    {
        // real robot provides binarized information on contacts
        if (!getContactsTactile(hand_name, fingers_contacts))
            return;
    }

    // extract contact points
    getContactPoints(fingers_contacts, fingers_pos, fingers_points);

    // print information
    for (auto it = fingers_contacts.begin();
         it != fingers_contacts.end();
         it++)
    {
        if (it->second)
        {
            yInfo() << "[TACTILE][" << hand_name << "hand]"
                    << it->first;
            yInfo() << "@"
                    << fingers_points.at(it->first).toString();
        }
    }

    return;
}

void LocalizerModule::stopFiltering()
{
    filtering_enabled = false;

    // reset flag for the next activation
    is_first_step = true;
}

void LocalizerModule::initFilters()
{
    upf0.init();
    upf1.init();
    // upf_pred.init();
    // upf_vis.init();

    // copy initial state
    // std::deque<ParticleUPF> particles;
    // upf0.getParticleSet(particles);
    // upf1.setParticleSet(particles);
    // upf_pred.setParticleSet(particles);
    // upf_vis.setParticleSet(particles);
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

    // same for the auxiliary estimate
    pose.transFromVec(last_aux_estimate[0],
                      last_aux_estimate[1],
                      last_aux_estimate[2]);

    // estimate is saved as x-y-z-Y-P-R
    pose.rotFromRPY(last_aux_estimate[5],
                    last_aux_estimate[4],
                    last_aux_estimate[3]);

    // Set a new transform
    tf_client->setTransform(aux_est_target_frame_name,
                            aux_est_source_frame_name,
                            pose.toMatrix());
}

bool LocalizerModule::retrieveGroundTruth(yarp::sig::Vector &pose)
{
    // Get the pose of the root frame of the robot
    yarp::sig::Matrix inertialToRobot(4,4);
    std::string source = gtruth_source_frame_name;
    std::string target = gtruth_target_frame_name;

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

yarp::sig::Matrix LocalizerModule::eulerZYX2dcm(const yarp::sig::Vector &euler)
{
    yarp::sig::Matrix dcm(3, 3);

    double phi = euler[0];
    double theta = euler[1];
    double psi = euler[2];
    dcm(0, 0) = cos(phi) * cos(theta);
    dcm(0, 1) = cos(phi) * sin(theta) * sin(psi)-sin(phi) * cos(psi);
    dcm(0, 2) = cos(phi) * sin(theta) * cos(psi)+sin(phi) * sin(psi);
    dcm(1, 0) = sin(phi) * cos(theta);
    dcm(1, 1) = sin(phi) * sin(theta) * sin(psi)+cos(phi) * cos(psi);
    dcm(1, 2) = sin(phi) * sin(theta) * cos(psi)-cos(phi) * sin(psi);
    dcm(2, 0) = -sin(theta);
    dcm(2, 1) = cos(theta) * sin(psi);
    dcm(2, 2) = cos(theta) * cos(psi);

    return dcm;
}

void LocalizerModule::evaluateVisualTactileMismatch(const yarp::sig::Vector &visual_estimate,
                                                    const yarp::sig::Vector &tactile_estimate,
                                                    yarp::sig::Matrix &mismatch)
{

    yarp::sig::Matrix rot;
    yarp::sig::Vector pos;

    // transformation due to visual data
    yarp::sig::Matrix T_vis(4,4);
    T_vis.zero();

    pos.resize(4, 0.0);
    pos.setSubvector(0, visual_estimate.subVector(0, 2));
    pos[3] = 1.0;
    T_vis.setCol(3, pos);
    rot = eulerZYX2dcm(visual_estimate.subVector(3, 5));
    T_vis.setSubmatrix(rot, 0, 0);

    // transformation due to tactile data
    yarp::sig::Matrix T_tac(4,4);
    T_tac.zero();

    pos.resize(4, 0.0);
    pos.setSubvector(0, tactile_estimate.subVector(0, 2));
    pos[3] = 1.0;
    T_tac.setCol(3, pos);
    rot = eulerZYX2dcm(tactile_estimate.subVector(3, 5));
    T_tac.setSubmatrix(rot, 0, 0);

    mismatch = SE3inv(T_tac) * T_vis;
    mismatch(0, 3) = mismatch(1, 3) = mismatch(2, 3) = 0.0;
}

void LocalizerModule::correctMeasurements(const yarp::sig::Vector &tactile_estimate,
                                          const yarp::sig::Matrix &vis_tac_mismatch,
                                          const std::vector<yarp::sig::Vector> &measurements,
                                          std::vector<yarp::sig::Vector> &corrected_measurements)
{
    // create correction matrix

    // transformation due to tactile data
    yarp::sig::Matrix T_tac(4, 4);
    T_tac.zero();

    yarp::sig::Vector pos(4, 0.0);
    pos.setSubvector(0, tactile_estimate.subVector(0, 2));
    pos[3] = 1.0;
    T_tac.setCol(3, pos);
    yarp::sig::Matrix rot = eulerZYX2dcm(tactile_estimate.subVector(3, 5));
    T_tac.setSubmatrix(rot, 0, 0);

    // change of coordinate from root frame to
    // object frame
    yarp::sig::Matrix base_change = T_tac;
    base_change(0, 3) = base_change(1, 3) = base_change(2, 3) = 0.0;
    base_change = base_change.transposed();

    // correction matrix
    yarp::sig::Matrix correction = T_tac * vis_tac_mismatch * base_change;

    // correct measurements
    for (size_t i=0; i<measurements.size(); i++)
    {
        yarp::sig::Vector meas_homog(4, 0.0);
        meas_homog.setSubvector(0, measurements[i] - tactile_estimate.subVector(0, 2));
        meas_homog[3] = 1.0;

        yarp::sig::Vector meas_corr(4, 0.0);
        meas_corr = correction  * meas_homog;
        corrected_measurements.push_back(meas_corr.subVector(0, 2));
    }
}

void LocalizerModule::resetStorage()
{
    // reset internal storage
    storage.clear();
}

Data& LocalizerModule::storeData(const FilteringType &data_type,
                                 const yarp::sig::Vector &ground_truth,
                                 const yarp::sig::Vector &estimate,
                                 const yarp::sig::Vector &corrected_est,
                                 const yarp::sig::Vector &vision_est,
                                 const yarp::sig::Vector &pred_est,
                                 const std::vector<yarp::sig::Vector> &particles,
                                 const std::vector<yarp::sig::Vector> &meas,
                                 const std::vector<yarp::sig::Vector> &corrected_meas,
                                 const yarp::sig::Vector &input,
                                 const double &time_stamp,
                                 const double &exec_time)
{
    Data d;

    // populate
    d.data_type = data_type;
    d.ground_truth = ground_truth;
    d.estimate = estimate;
    d.corrected_est = corrected_est;
    d.vision_est = vision_est;
    d.pred_est = pred_est;
    d.particles = particles;
    d.meas = meas;
    d.corrected_meas = corrected_meas;
    d.input = input;
    d.time_stamp = time_stamp;
    d.exec_time = exec_time;

    // add to storage
    storage.push_back(d);

    // return reference to last element
    return storage.back();
}

void LocalizerModule::storeDataVisual(const FilteringType &data_type,
                                      const bool &is_first_chunk,
                                      const yarp::sig::Vector &ground_truth,
                                      const yarp::sig::Vector &estimate,
                                      const yarp::sig::Vector &corrected_est,
                                      const yarp::sig::Vector &vision_est,
                                      const yarp::sig::Vector &pred_est,
                                      const std::vector<yarp::sig::Vector> &particles,
                                      const std::vector<yarp::sig::Vector> &meas,
                                      const yarp::sig::Vector &input,
                                      const std::vector<yarp::sig::Vector> &filtered_pc,
                                      const std::vector<yarp::sig::Vector> &true_pc,
                                      const double &time_stamp,
                                      const double &exec_time)
{
    // store common data
    Data &d = storeData(FilteringType::visual, ground_truth,
                        estimate, corrected_est, vision_est, pred_est,
                        particles,
                        meas, meas, input, time_stamp, exec_time);

    // add additional fields
    d.is_first_chunk = is_first_chunk;
    d.filtered_pc = filtered_pc;
    d.true_pc = true_pc;
}

void LocalizerModule::storeDataTactile(const yarp::sig::Vector &ground_truth,
                                       const yarp::sig::Vector &estimate,
                                       const yarp::sig::Vector &corrected_est,
                                       const yarp::sig::Vector &vision_est,
                                       const yarp::sig::Vector &pred_est,
                                       const std::vector<yarp::sig::Vector> &particles,
                                       const std::vector<yarp::sig::Vector> &meas,
                                       const std::vector<yarp::sig::Vector> &corrected_meas,
                                       const yarp::sig::Vector &input,
                                       std::unordered_map<std::string, yarp::sig::Vector> fingers_joints,
                                       std::unordered_map<std::string, yarp::sig::Vector> fingers_pos,
                                       std::unordered_map<std::string, yarp::sig::Vector> fingers_vels,
                                       std::unordered_map<std::string, bool> contacts_tactile,
                                       std::unordered_map<std::string, bool> contacts_springy,
                                       const double &time_stamp,
                                       const double &exec_time)
{
    // store common data
    Data &d = storeData(FilteringType::tactile, ground_truth,
                        estimate, corrected_est, vision_est, pred_est,
                        particles,
                        meas, corrected_meas, input, time_stamp, exec_time);

    // add additional fields
    d.fingers_joints = fingers_joints;
    d.fingers_pos = fingers_pos;
    d.fingers_vels = fingers_vels;
    d.contacts_tactile = contacts_tactile;
    d.contacts_springy = contacts_springy;
}

bool LocalizerModule::saveMesh(const yarp::sig::Vector &pose,
                               const std::string &file_name)
{
    // obtain the mesh from the filter
    // (the filter contains the model of the object)
    Polyhedron p;
    upf0.transformObject(pose, p);

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
            else if (finger_name == "little")
                fout << 4;
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
            else if (finger_name == "little")
                fout << 4;
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
            else if (finger_name == "little")
                fout << 4;
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

bool LocalizerModule::saveContacts(const std::unordered_map<std::string, bool> &contacts_tactile,
                                   const std::unordered_map<std::string, bool> &contacts_springy,
                                   const std::string &file_name)
{
    // save the contacts
    // overwrite if it already exists
    std::ofstream fout(file_name.c_str(), std::ios::trunc);
    if(fout.is_open())
    {
        // print the CSV header
        fout << "finger_id;"
             << "is_tactile;" << "is_springy;"
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
            else if (finger_name == "little")
                fout << 4;
            else
                return false;
            fout << ";";

            // write to file
            fout << contacts_tactile.at(finger_name) << ";"
                 << contacts_springy.at(finger_name) << ";"
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


bool LocalizerModule::saveParticles(const std::vector<yarp::sig::Vector> &particles,
                                    const std::string &file_name)
{
    // save the particles
    // overwrite if it already exists
    std::ofstream fout(file_name.c_str(), std::ios::trunc);
    if(fout.is_open())
    {
        // print the CSV header
        fout << "x;" << "y;" << "z;"
             << "phi;" << "theta;" << "psi;"
             << std::endl;

        for (const yarp::sig::Vector &particle : particles)
        {
            // write to file
            fout << particle[0] << ";"
                 << particle[1] << ";"
                 << particle[2] << ";"
                 << particle[3] << ";"
                 << particle[4] << ";"
                 << particle[5] << ";"
                 << std::endl;
        }
    }
    else
    {
        fout.close();

        yError() << "LocalizerModule: problem opening particles output file"
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
             << "x_aux;"   << "y_aux;"     << "z_aux;"
             << "phi_aux;" << "theta_aux;" << "psi_aux;"
             << "x_pred;"   << "y_pred;"     << "z_pred;"
             << "phi_pred;" << "theta_pred;" << "psi_pred;"
             << "x_vis;"   << "y_vis;"     << "z_vis;"
             << "phi_vis;" << "theta_vis;" << "psi_vis;"
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
            // aux estimate
            for(size_t j=0; j<6; j++)
                fout << d.corrected_est[j] << ";";
            // prediction only estimate
            for(size_t j=0; j<6; j++)
                fout << d.pred_est[j] << ";";
            // vision only estimate
            for(size_t j=0; j<6; j++)
                fout << d.vision_est[j] << ";";
            // input
            for(size_t j=0; j<3; j++)
                fout << d.input[j] << ";";
            // time stamp
            fout << std::fixed;
            fout << d.time_stamp << ";";
            fout << std::defaultfloat;
            // execution time
            fout << d.exec_time << ";";

            fout << std::endl;

            // save all the particles for each step
            std::string particles_path = output_path + "particles_step_" +
                std::to_string(step_index) + ".csv";
            if (!saveParticles(d.particles, particles_path))
            {
                // error message is provided by saveParticles()
                fout.close();
                return false;
            }

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
            meas_path = output_path + "corr_meas_step_"
                + std::to_string(step_index) + ".off";
            if (!saveMeas(d.corrected_meas, meas_path))
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

                // save fingers velocities
                std::string contacts_path = output_path + "fingers_contacts_"
                    + std::to_string(step_index) + ".csv";
                if (!saveContacts(d.contacts_tactile, d.contacts_springy, contacts_path))
                {
                    fout.close();
                    return false;
                }
            }
	    else if (d.data_type == FilteringType::visual)
	    {
		// save filtered point cloud
		// effectively used by the filter
		std::string filtered_pc_path = output_path + "filtered_pc_step_"
		    + std::to_string(step_index) +  ".off";
		if (!saveMeas(d.filtered_pc, filtered_pc_path))
		{
		    // error message is provided by saveMeas()
		    fout.close();
		    return false;
		}

		// save true point cloud from stereo vision
		std::string pc_path = output_path + "pc_step_"
		    + std::to_string(step_index) +  ".off";
		if (!saveMeas(d.true_pc, pc_path))
		{
		    // error message is provided by saveMeas()
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
        reply.addString("- visual-on");
        reply.addString("- tactile-on <hand_name>");
        reply.addString("- filter-off");
        reply.addString("- pc-subsample <on/off> <n_points>");
        reply.addString("- pc-shuffle <on/off> <resize_factor>");
        reply.addString("- pc-out-removal <on/off> <radius> <neigh>");
        reply.addString("- pc-dense-out-removal <on/off> <thr>");
        reply.addString("- storage-on");
        reply.addString("- storage-off");
        reply.addString("- storage-save");
        reply.addString("- calibrate-hand");
        reply.addString("- reset");
        reply.addString("- quit");
    }
    else if (cmd == "visual-on")
    {
        mutex.lock();

        filtering_enabled = true;
        filtering_type = FilteringType::visual;

        mutex.unlock();

        reply.addString("Visual filtering enabled succesfully.");
    }
    else if (cmd == "tactile-on")
    {
        yarp::os::Value hand_name_value;
        hand_name_value = command.get(1);

        if (hand_name_value.isNull())
            reply.addString("Hand name is required.");
        else if (!hand_name_value.isString())
            reply.addString("A valid hand name is required.");
        else
        {
            std::string hand_name_ = hand_name_value.asString();

            if ((hand_name != "right") && (hand_name != "left"))
                reply.addString("A valid hand name is required.");
            else
            {
                mutex.lock();

                hand_name = hand_name_;
                filtering_enabled = true;
                filtering_type = FilteringType::tactile;

                mutex.unlock();

                reply.addString("Tactile filtering enabled succesfully.");
            }
        }
    }
    else if (cmd == "filter-off")
    {
        mutex.lock();

        filtering_enabled = false;

        mutex.unlock();

        reply.addString("Filtering disabled succesfully.");
    }
    else if (cmd == "pc-subsample")
    {
        mutex.lock();

        bool valid = true;

        if (command.size() != 3)
            valid = false;

        yarp::os::Value enable = command.get(1);
        if ((enable.isNull()) || (!enable.isString()) ||
            ((enable.asString() != "on") && (enable.asString() != "off")))
            valid = false;

        yarp::os::Value num_points = command.get(2);
        if ((num_points.isNull()) || (!num_points.isInt()))
            valid = false;

        if (!valid)
            reply.addString("Invalid request.");
        else
        {
            std::string enable_string = enable.asString();
            use_pc_subsampling = (enable_string == "on");

            subsample_n_points = num_points.asInt();

            reply.addString("Settings for point cloud subsampling accepted.");
        }

        mutex.unlock();
    }
    else if (cmd == "pc-shuffle")
    {
        mutex.lock();

        bool valid = true;

        if (command.size() != 3)
            valid = false;

        yarp::os::Value enable = command.get(1);
        if ((enable.isNull()) || (!enable.isString()) ||
            ((enable.asString() != "on") && (enable.asString() != "off")))
            valid = false;

        yarp::os::Value resize_factor = command.get(2);
        if ((resize_factor.isNull()) || (!resize_factor.isDouble()))
            valid = false;

        if (!valid)
            reply.addString("Invalid request.");
        else
        {
            std::string enable_string = enable.asString();
            use_pc_shuffle = (enable_string == "on");

            shuffle_resize_factor = resize_factor.asDouble();

            reply.addString("Settings for point cloud shuffling accepted.");
        }

        mutex.unlock();
    }
    else if (cmd == "pc-out-removal")
    {
        mutex.lock();

        bool valid = true;

        if (command.size() != 4)
            valid = false;

        yarp::os::Value enable = command.get(1);
        if ((enable.isNull()) || (!enable.isString()) ||
            ((enable.asString() != "on") && (enable.asString() != "off")))
            valid = false;

        yarp::os::Value radius = command.get(2);
        if ((radius.isNull()) || (!radius.isDouble()))
            valid = false;

        yarp::os::Value neigh = command.get(3);
        if ((neigh.isNull()) || (!neigh.isInt()))
            valid = false;

        if (!valid)
            reply.addString("Invalid request.");
        else
        {
            std::string enable_string = enable.asString();
            use_pc_outlier_rem = (enable_string == "on");

            outlier_rem_radius = radius.asDouble();
            outlier_rem_neigh = neigh.asInt();

            reply.addString("Settings for point cloud outlier removal accepted.");
        }

        mutex.unlock();
    }
    else if (cmd == "pc-dense-out-removal")
    {
        mutex.lock();

        bool valid = true;

        if (command.size() != 3)
            valid = false;

        yarp::os::Value enable = command.get(1);
        if ((enable.isNull()) || (!enable.isString()) ||
            ((enable.asString() != "on") && (enable.asString() != "off")))
            valid = false;

        yarp::os::Value threshold = command.get(2);
        if ((threshold.isNull()) || (!threshold.isDouble()))
            valid = false;

        if (!valid)
            reply.addString("Invalid request.");
        else
        {
            std::string enable_string = enable.asString();
            use_pc_dense_outlier_rem = (enable_string == "on");

            dense_outlier_rem_thr = threshold.asDouble();

            reply.addString("Settings for point cloud dense outlier removal accepted.");
        }

        mutex.unlock();
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
    else if (cmd == "calibrate-hand")
    {
        bool ok = true;

        yarp::os::Value hand_name_v = command.get(1);
        if ((hand_name_v.isNull()) || (!hand_name_v.isString()) ||
            ((hand_name_v.asString() != "right") && (hand_name_v.asString() != "left")))
        {
            reply.addString("You should specify a valid hand name.");
            ok = false;
        }

        if (ok)
        {
            if (use_analogs && use_analogs_bounds)
            {
                ok = calibrateHand(hand_name_v.asString());
                if (ok)
                    reply.addString("Calibration done succesfully.");
                else
                    reply.addString("Calibration failed.");
            }
            else
                reply.addString("Warning: Use of analogs bounds is disabled from the configuration file.");
        }
    }
    else if (cmd == "reset")
    {
        // reset the filter
        initFilters();

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
    propTfClient.put("device", "transformClient");
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

    if (is_simulation)
    {
    // get the pose of the root frame of the robot
    // required to convert point clouds in simulation
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
    }

    // prepare properties for the Encoders
    yarp::os::Property prop_encoders;
    prop_encoders.put("device", "remote_controlboard");
    prop_encoders.put("remote", "/" + robot_name + "/right_arm");
    prop_encoders.put("local", "/upf-localizer/encoders/right_arm");
    ok_drv = drv_right_arm.open(prop_encoders);
    if (!ok_drv)
    {
        yError() << "LocalizerModule::configure error:"
                 << "unable to open the Remote Control Board driver for the right arm";
        return false;
    }

    prop_encoders.put("remote", "/" + robot_name + "/left_arm");
    prop_encoders.put("local", "/upf-localizer/encoders/left_arm");
    ok_drv = drv_left_arm.open(prop_encoders);
    if (!ok_drv)
    {
        yError() << "LocalizerModule::configure error:"
                 << "unable to open the Remote Control Board driver for the left arm";
        return false;
    }

    prop_encoders.put("remote", "/" + robot_name + "/torso");
    prop_encoders.put("local", "/upf-localizer/encoders/torso");
    ok_drv = drv_torso.open(prop_encoders);
    if (!ok_drv)
    {
        yError() << "LocalizerModule::configure error:"
                 << "unable to open the Remote Control Board driver for the torso";
        return false;
    }

    if (use_analogs)
    {
        prop_encoders.put("device", "analogsensorclient");
        prop_encoders.put("remote", "/" + robot_name + "/right_hand/analog:o");
        prop_encoders.put("local", "/upf-localizer/analogs/right_hand");
        ok_drv = drv_right_analog.open(prop_encoders);
        if (!ok_drv)
        {
            yError() << "LocalizerModule::configure error:"
                     << "unable to open the Analog Sensor Client driver for the right hand";
            return false;
        }

        prop_encoders.put("device", "analogsensorclient");
        prop_encoders.put("remote", "/" + robot_name + "/left_hand/analog:o");
        prop_encoders.put("local", "/upf-localizer/analogs/left_hand");
        ok_drv = drv_left_analog.open(prop_encoders);
        if (!ok_drv)
        {
            yError() << "LocalizerModule::configure error:"
                     << "unable to open the Analog Sensor Client driver for the left hand";
            return false;
        }
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
    if (use_analogs)
    {
        ok_view = drv_right_analog.view(ianalog_right);
        if (!ok_view || ianalog_right == 0)
        {
            yError() << "LocalizerModule:configure error:"
                     << "unable to retrieve the Analogs view for the right hand";
            return false;
        }
        ok_view = drv_left_analog.view(ianalog_left);
        if (!ok_view || ianalog_left == 0)
        {
            yError() << "LocalizerModule:configure error:"
                     << "unable to retrieve the Analogs view for the left hand";
            return false;
        }
        ok_view = drv_right_arm.view(ilim_right);
        if (!ok_view || ilim_right == 0)
        {
            yError() << "LocalizerModule:configure error:"
                     << "unable to retrieve the IControlLimits view for the right hand";
            return false;
        }
        ok_view = drv_left_arm.view(ilim_left);
        if (!ok_view || ilim_left == 0)
        {
            yError() << "LocalizerModule:configure error:"
                     << "unable to retrieve the IControlLimits view for the left hand";
            return false;
        }
    }

    // configure arms forward kinematics
    right_arm_kin = iCub::iKin::iCubArm("right");
    left_arm_kin = iCub::iKin::iCubArm("left");

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

    // configure finger forward kinematics
    bool ok;
    std::deque<yarp::dev::IControlLimits*> lims_right;
    std::deque<yarp::dev::IControlLimits*> lims_left;
    if (use_analogs)
    {
        lims_right.push_back(ilim_right);
        lims_left.push_back(ilim_left);
    }

    fingers_names = {"thumb", "index", "middle", "ring", "little"};
    for (std::string &finger_name : fingers_names)
    {
        std::string right_finger = "right_" + finger_name;
        std::string left_finger = "left_" + finger_name;
        std::string right_finger_key = right_finger;
        std::string left_finger_key = left_finger;
        fingers_kin[right_finger_key] = iCub::iKin::iCubFingerExt(right_finger);
        fingers_kin[left_finger_key] = iCub::iKin::iCubFingerExt(left_finger);

        if (use_analogs)
        {
            ok = fingers_kin[right_finger_key].alignJointsBounds(lims_right);
            if (!ok)
            {
                yError() << "Localizer module: cannot set joints bounds for finger"
                         << right_finger;
                return false;
            }
            ok = fingers_kin[left_finger_key].alignJointsBounds(lims_left);
            if (!ok)
            {
                yError() << "Localizer module: cannot set joints bounds for finger"
                         << left_finger;
            }

            // fix limits for thumb opposition
            iCub::iKin::iKinChain &left_thumb = fingers_kin["left_thumb"];
            left_thumb[0].setMin((-15.0) * iCub::ctrl::CTRL_DEG2RAD);
            // fix limits for thumb opposition
            iCub::iKin::iKinChain &right_thumb = fingers_kin["right_thumb"];
            right_thumb[0].setMin((-15.0) * iCub::ctrl::CTRL_DEG2RAD);
        }
    }
    // setup matrix containings
    // analog bounds for encoders of fingers proximal/distal joints
    setupAnalogBounds();

    // configure joints velocity estimator
    if (use_ext_vel_observer)
    {
        joints_vel_estimator = std::unique_ptr<iCub::ctrl::AWLinEstimator>(
            new iCub::ctrl::AWLinEstimator(20, 2.0));
    }

    // configure springy fingers
    std::string left_springy_calib_path;
    std::string right_springy_calib_path;
    if (!rf_module.check("springyFingersCalibLeft"))
    {
        yError() << "Localizer module: cannot load path containing"
                 << "the calibration file for left springy fingers";
        return false;
    }
    left_springy_calib_path = rf_module.findFile("springyFingersCalibLeft");
    if (!rf_module.check("springyFingersCalibRight"))
    {
        yError() << "Localizer module: cannot load path containing"
                 << "the calibration file for right springy fingers";
        return false;
    }
    right_springy_calib_path = rf_module.findFile("springyFingersCalibRight");
    yarp::os::Property left_springy_prop;
    yarp::os::Property right_springy_prop;
    left_springy_prop.fromConfigFile(left_springy_calib_path.c_str());
    right_springy_prop.fromConfigFile(right_springy_calib_path.c_str());
    left_springy_prop.put("robot", robot_name.c_str());
    right_springy_prop.put("robot", robot_name.c_str());
    left_springy_fingers.fromProperty(left_springy_prop);
    right_springy_fingers.fromProperty(right_springy_prop);
    if (!left_springy_fingers.isCalibrated())
    {
        yError() << "Localizer module: cannot configure"
                 << "left springy fingers";
        return false;
    }
    if (!right_springy_fingers.isCalibrated())
    {
        yError() << "Localizer module: cannot configure"
                 << "right springy fingers";
        return false;
    }

    // configure and init the UPF
    // using group 'upf' from the configuration file
    yarp::os::ResourceFinder rf_upf;
    rf_upf = rf.findNestedResourceFinder("upf");
    if((!upf0.configure(rf_upf))     || (!upf1.configure(rf_upf)) ||
       (!upf_pred.configure(rf_upf)) || (!upf_vis.configure(rf_upf)))
        return false;
    initFilters();

    // reset storage
    storage_on = false;
    resetStorage();

    // start rpc server
    rpc_port.open(rpc_port_name);
    attach(rpc_port);

    // reset flags
    estimate_available = false;
    filtering_enabled = false;
    contacts_probe_enabled = false;
    is_first_step = true;
    is_vis_tac_mismatch = false;

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

    // do contacts probe
    performContactsProbe();

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
    drv_left_analog.close();
    drv_right_analog.close();
}

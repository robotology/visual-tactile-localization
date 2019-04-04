/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ICUBHANDOCCLUSION_H
#define ICUBHANDOCCLUSION_H

#include <GazeController.h>
#include <iCubArmModel.h>
#include <ObjectOcclusion.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

#include <Eigen/Dense>

class iCubHandOcclusion : public ObjectOcclusion
{
public:
    iCubHandOcclusion(std::unique_ptr<iCubArmModel> icub_arm_model, const std::string port_prefix, const std::string eye_name, const double occlusion_scale);

    virtual ~iCubHandOcclusion();

    std::pair<bool, Eigen::MatrixXd> getOcclusionPose() override;

    std::tuple<bool, Eigen::VectorXd, Eigen::VectorXd> getCameraPose() override;

private:
    yarp::os::BufferedPort<yarp::sig::Vector> hand_pose_port_in;

    const std::string log_ID_ = "[ICUBHANDOCCLUSION]";

    GazeController gaze_;

    const double icub_cam_width_ = 320;

    const double icub_cam_height_ = 240;

    const std::string eye_name_;
};

#endif /* ICUBHANDCCLUSION_H */

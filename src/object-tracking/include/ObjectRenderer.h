/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef OBJECTRENDERER_H
#define OBJECTRENDERER_H

#include <Camera.h>

#include <Eigen/Dense>

#include <SuperimposeMesh/SICAD.h>

#include <opencv2/opencv.hpp>

#include <string>


class ObjectRenderer
{
public:

    ObjectRenderer(const std::string& object_mesh_path, const std::string& sicad_shader_path, Camera& camera);

    virtual ~ObjectRenderer();

    std::pair<bool, cv::Mat> renderObject(const Eigen::Transform<double, 3, Eigen::Affine>& object_pose, const Eigen::Transform<double, 3, Eigen::Affine>& camera_pose);

protected:

    std::unique_ptr<SICAD> object_sicad_;

    const std::string log_ID_ = "ObjectRenderer";
};

#endif

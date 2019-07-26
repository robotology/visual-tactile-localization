/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef SINGLEVIDEORESULT_H
#define SINGLEVIDEORESULT_H

#include <Eigen/Dense>

class SingleVideoResult
{
public:
    SingleVideoResult(const std::string& object_name, const std::string& video_name, const std::string& data_path, const std::string& result_path);

    Eigen::Transform<double, 3, Eigen::Affine> getObjectPose(const std::size_t& frame_id);

    Eigen::Transform<double, 3, Eigen::Affine> getGroundTruthPose(const std::size_t& frame_id);

private:
    std::pair<bool, Eigen::MatrixXd> readDataFromFile(const std::string& filename, const std::size_t num_fields);

    Eigen::Transform<double, 3, Eigen::Affine> makeTransform(const Eigen::VectorXd pose);

    Eigen::MatrixXd estimate_;

    Eigen::MatrixXd ground_truth_;

    const std::string log_ID_ = "[SingleVideoResult]";
};

#endif /* SINGLEVIDEORESULT_H */

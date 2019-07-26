/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <SingleVideoResult.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace Eigen;


SingleVideoResult::SingleVideoResult(const std::string& object_name, const std::string& video_name, const std::string& data_path, const std::string& result_path)
{
    /* Compose root paths. */
    std::string root_data = data_path;
    if (root_data.back() != '/')
        root_data += '/';

    std::string root_result = result_path;
    if (root_result.back() != '/')
        root_result += '/';

    /* Compose estimate file name. */
    std::string estimate_file_name = root_result + object_name + "/" + video_name + "/object-tracking_estimate.txt";

    /* Compose ground truth file name. */
    std::string ground_truth_file_name = root_data + video_name + "/output/gt_" + object_name.substr(object_name.find("_") + 1) + "/data.log";

    /* Load estimate. */
    bool valid_estimate = false;
    std::tie(valid_estimate, estimate_) = readDataFromFile(estimate_file_name, 14);
    if (!valid_estimate)
    {
        std::string error = log_ID_ + "::ctor. Error: cannot load results from file " + estimate_file_name;
        throw(std::runtime_error(error));
    }

    /* Load ground truth. */
    bool valid_ground_truth = false;
    std::tie(valid_ground_truth, ground_truth_) = readDataFromFile(ground_truth_file_name, 9);
    if (!valid_ground_truth)
    {
        std::string error = log_ID_ + "::ctor. Error: cannot load ground truth from file " + ground_truth_file_name;
        throw(std::runtime_error(error));
    }
}


Transform<double, 3, Affine> SingleVideoResult::getObjectPose(const std::size_t& frame_id)
{
    // frame_id starts from 1 in YCB Video dataset
    VectorXd vector = estimate_.col(frame_id - 1);

    return makeTransform(vector.segment<7>(0));
}


Transform<double, 3, Affine> SingleVideoResult::getGroundTruthPose(const std::size_t& frame_id)
{
    // frame_id starts from 1 in YCB Video dataset
    VectorXd vector = ground_truth_.col(frame_id - 1);

    return makeTransform(vector.segment<7>(2));
}


std::pair<bool, MatrixXd> SingleVideoResult::readDataFromFile(const std::string& filename, const std::size_t num_fields)
{
    MatrixXd data;

    std::ifstream istrm(filename);

    if (!istrm.is_open())
    {
        std::cout << log_ID_ + "::readDataFromFile. Error: failed to open " << filename << '\n';

        istrm.close();

        return std::make_pair(false, MatrixXd(0,0));
    }
 
    std::vector<std::string> istrm_strings;
    std::string line;
    while (std::getline(istrm, line))
    {
        istrm_strings.push_back(line);
    }

    data.resize(num_fields, istrm_strings.size());
    std::size_t found_lines = 0;
    for (auto line : istrm_strings)
    {
        std::size_t found_fields = 0;
        std::string number_str;
        std::istringstream iss(line);

        while (iss >> number_str)
        {
            std::size_t index = (num_fields * found_lines) + found_fields;
            *(data.data() + index) = std::stod(number_str);
            found_fields++;
        }
        if (num_fields != found_fields)
        {
            std::cout << log_ID_ + "::readDataFromFile. Error: malformed input file " << filename << '\n';

            return std::make_pair(false, MatrixXd(0,0));
        }
        found_lines++;
    }

    istrm.close();

    return std::make_pair(true, data);
}

Transform<double, 3, Affine> SingleVideoResult::makeTransform(const VectorXd pose)
{
    /**
     * pose expected to be a 7-dimensional vector containing
     * x - y - z - axis vector - angle
     */
    
    Transform<double, 3, Affine> transform;

    /* Compose translational part. */
    transform = Translation<double, 3>(pose.head<3>());

    /* Compose rotational part. */
    AngleAxisd rotation(pose(6), pose.segment<3>(3));
    transform.rotate(rotation);

    return transform;
}

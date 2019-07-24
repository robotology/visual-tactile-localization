/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <InitParticlesGroundTruth.h>

#include <fstream>
#include <iostream>


using namespace bfl;
using namespace Eigen;


InitParticlesGroundTruth::InitParticlesGroundTruth
(
    const std::string& path,
    const std::string& object_name,
    const Ref<const MatrixXd>& initial_covariance
) :
    initial_covariance_(initial_covariance)
{
    /* Compose root path. */
    std::string root = path;
    if (root.back() != '/')
        root += '/';

    /* Compose path containing ground truth data. */
    std::string file_name = root + "gt_" + object_name.substr(object_name.find("_") + 1) + "/data.log";

    /* Load data. */
    bool valid_data = false;
    MatrixXd data;
    std::tie(valid_data, data) = readStateFromFile(file_name, 9);
    if (!valid_data)
    {
        std::string error = log_ID_ + "::ctor. Error: cannot open file " + file_name;
        throw(std::runtime_error(error));
    }

    /* Extract initial pose and convert to x-y-z-Euler(Z, Y, X). */
    initial_pose_ = VectorXd::Zero(12);
    initial_pose_.head<3>() = data.col(0).segment<3>(2);
    AngleAxisd angle_axis(data.col(0)(8), data.col(0).segment<3>(5));
    initial_pose_.tail<3>() = angle_axis.toRotationMatrix().eulerAngles(2, 1, 0);
}


InitParticlesGroundTruth::~InitParticlesGroundTruth() noexcept
{ }


bool InitParticlesGroundTruth::initialize(ParticleSet& particles)
{
    for (int i = 0; i < particles.state().cols(); ++i)
    {
        particles.mean(i) = initial_pose_;

        // Initialize state covariance
        particles.covariance(i) = initial_covariance_;
    }

    // Set particles position the same as the mean state
    // TODO: they should be sampled from the initial gaussian
    particles.state() = particles.mean();

    // Initialize weights
    particles.weight().fill(-std::log(particles.state().cols()));

    return true;
}


std::pair<bool, MatrixXd> InitParticlesGroundTruth::readStateFromFile(const std::string& filename, const std::size_t num_fields)
{
    MatrixXd data;

    std::ifstream istrm(filename);

    if (!istrm.is_open())
    {
        std::cout << "Failed to open " << filename << '\n';

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
            std::cout << "Malformed input file " << filename << '\n';

            return std::make_pair(false, MatrixXd(0,0));
        }
        found_lines++;
    }

    return std::make_pair(true, data);
}

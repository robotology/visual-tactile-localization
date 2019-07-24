/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <Viewer.h>

int main(int argc, char* argv[])
{
    // Parse command line arguments
    if(argc != 7)
    {
        std::cout << "Usage: " << argv[0]
                  << " <use_ground_truth> (true/false)"
                  << " Filename(.obj)"
                  << " Trajectory"
                  << " Estimate"
                  << " Prediction"
                  << " Measurements" << std::endl;
        return EXIT_FAILURE;
    }

    // Start visualizer
    std::string use_ground_truth = argv[1];
    std::string mesh_filename = argv[2];
    std::string target_filename = argv[3];
    std::string estimate_filename = argv[4];
    std::string prediction_filename = argv[5];
    std::string measurements_filename = argv[6];
    Visualizer(mesh_filename, target_filename, estimate_filename, prediction_filename, measurements_filename, use_ground_truth);

    return EXIT_SUCCESS;
}



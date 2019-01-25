#include <Viewer.h>

int main(int argc, char* argv[])
{
    // Parse command line arguments
    if(argc != 7)
    {
        std::cout << "Usage: " << argv[0]
                  << " <use_ground_truth> (true/false)"
                  << " Filename(.ply)"
                  << " Trajectory(.txt)"
                  << " Estimate(.txt)"
                  << " Prediction(.txt)"
                  << " Measurements(.txt)" << std::endl;
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



#include <Viewer.h>

int main(int argc, char* argv[])
{
    // Parse command line arguments
    if(argc != 6)
    {
        std::cout << "Usage: " << argv[0]
                  << " Filename(.ply)"
                  << " Trajectory(.txt)"
                  << " Estimate(.txt)"
                  << " Prediction(.txt)"
                  << " Measurements(.txt)" << std::endl;
        return EXIT_FAILURE;
    }

    // Start visualizer
    std::string mesh_filename = argv[1];
    std::string target_filename = argv[2];
    std::string estimate_filename = argv[3];
    std::string prediction_filename = argv[4];
    std::string measurements_filename = argv[5];
    Visualizer(mesh_filename, target_filename, estimate_filename, prediction_filename, measurements_filename);

    return EXIT_SUCCESS;
}



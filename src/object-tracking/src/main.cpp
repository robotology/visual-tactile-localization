#include <Correction.h>
#include <iCubPointCloud.h>
#include <KinematicModel.h>
#include <NanoflannPointCloudPrediction.h>
#include <Random3DPose.h>
#include <SimulatedFilter.h>
#include <SimulatedPointCloud.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>

#include <Eigen/Dense>

#include <cstdlib>
#include <string>
#include <sstream>

using namespace yarp::os;
using namespace Eigen;

VectorXd loadVectorDouble
(
    ResourceFinder &rf,
    const std::string key,
    const std::size_t size
)
{
    bool ok = true;

    if (rf.find(key).isNull())
        ok = false;

    Bottle* b = rf.find(key).asList();
    if (b == nullptr)
        ok = false;

    if (b->size() != size)
        ok = false;

    if (!ok)
    {
        yError() << "[Main]" << "Unable to load vector" << key;
        std::exit(EXIT_FAILURE);
    }

    VectorXd vector(size);
    for (std::size_t i = 0; i < b->size(); i++)
    {
        Value item_v = b->get(i);
        if (item_v.isNull())
            return VectorXd(0);

        if (!item_v.isDouble())
            return VectorXd(0);

        vector(i) = item_v.asDouble();
    }

    return vector;
}


std::string eigenToString(VectorXd& v)
{
    std::stringstream ss;
    ss << v.transpose();
    return ss.str();
}


int main(int argc, char** argv)
{
    const std::string log_ID = "[Main]";
    yInfo() << log_ID << "Configuring and starting module...";

    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError() << log_ID << "YARP seems unavailable!";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.setVerbose();
    rf.setDefaultContext("object-tracking");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc, argv);

    /* Get mode of operation. */
    ResourceFinder mode_parameters = rf.findNestedResourceFinder("MODE");
    const std::string mode  = mode_parameters.check("mode",  Value("simulation")).asString();
    const std::string robot = mode_parameters.check("robot", Value("icubSim")).asString();

    /* Get initial condition. */
    ResourceFinder initial_conditions = rf.findNestedResourceFinder("INITIAL_CONDITION");
    VectorXd x_0           = loadVectorDouble(initial_conditions, "x_0",           3);
    VectorXd v_0           = loadVectorDouble(initial_conditions, "v_0",           3);
    VectorXd euler_0       = loadVectorDouble(initial_conditions, "euler_0",       3);
    VectorXd euler_dot_0   = loadVectorDouble(initial_conditions, "euler_dot_0",   3);
    VectorXd cov_x_0       = loadVectorDouble(initial_conditions, "cov_x_0",       3);
    VectorXd cov_v_0       = loadVectorDouble(initial_conditions, "cov_v_0",       3);
    VectorXd cov_eul_0     = loadVectorDouble(initial_conditions, "cov_eul_0",     3);
    VectorXd cov_eul_dot_0 = loadVectorDouble(initial_conditions, "cov_eul_dot_0", 3);

    /* Kinematic model. */
    ResourceFinder kinematic_model = rf.findNestedResourceFinder("KINEMATIC_MODEL");
    double sample_time     = kinematic_model.check("sample_time", Value("1.0")).asDouble();
    VectorXd kin_psd_acc   = loadVectorDouble(kinematic_model, "psd_acc", 3);
    VectorXd kin_psd_euler = loadVectorDouble(kinematic_model, "psd_euler_acc", 3);

    /* Measurement model. */
    ResourceFinder measurement_model = rf.findNestedResourceFinder("MEASUREMENT_MODEL");
    VectorXd noise_covariance = loadVectorDouble(measurement_model, "noise_covariance", 3);

    /* Unscented transform. */
    ResourceFinder unscented_transform = rf.findNestedResourceFinder("UNSCENTED_TRANSFORM");
    double ut_alpha = unscented_transform.check("alpha", Value("1.0")).asDouble();
    double ut_beta  = unscented_transform.check("beta", Value("2.0")).asDouble();
    double ut_kappa = unscented_transform.check("kappa", Value("0.0")).asDouble();

    /* Point cloud prediction. */
    ResourceFinder point_cloud_prediction = rf.findNestedResourceFinder("POINT_CLOUD_PREDICTION");
    std::size_t pc_pred_num_samples = point_cloud_prediction.check("number_samples", Value("100")).asInt();

    double sim_sample_time;
    double sim_duration;
    VectorXd sim_psd_acc;
    VectorXd sim_psd_w;
    VectorXd sim_x_0;
    VectorXd sim_v_0;
    VectorXd sim_q_0;
    VectorXd sim_w_0;
    //
    std::size_t sim_point_cloud_number;
    VectorXd sim_point_cloud_observer;
    VectorXd sim_point_cloud_noise_std;
    bool sim_point_cloud_back_culling;
    if (mode == "simulation")
    {
        ResourceFinder simulation = rf.findNestedResourceFinder("SIMULATION");
        sim_sample_time = simulation.check("sample_time", Value("1.0")).asDouble();
        sim_duration    = simulation.check("duration", Value("1.0")).asDouble();
        sim_psd_acc     = loadVectorDouble(simulation, "psd_acc", 3);
        sim_psd_w       = loadVectorDouble(simulation, "psd_w", 3);
        sim_x_0         = loadVectorDouble(simulation, "x_0", 3);
        sim_v_0         = loadVectorDouble(simulation, "v_0", 3);
        sim_q_0         = loadVectorDouble(simulation, "q_0", 4);
        sim_w_0         = loadVectorDouble(simulation, "w_0", 3);

        ResourceFinder sim_point_cloud = rf.findNestedResourceFinder("SIMULATED_POINT_CLOUD");
        sim_point_cloud_number       = sim_point_cloud.check("number_points", Value("1000")).asInt();
        sim_point_cloud_observer     = loadVectorDouble(sim_point_cloud, "observer_origin", 3);
        sim_point_cloud_noise_std    = loadVectorDouble(sim_point_cloud, "noise_std", 3);
        sim_point_cloud_back_culling = sim_point_cloud.check("back_cullnig", Value(true)).asBool();
    }

    /* Log parameters. */
    yInfo() << log_ID << "Mode of operation:";
    yInfo() << log_ID << "- mode:"  << mode;
    yInfo() << log_ID << "- robot:" << robot;

    yInfo() << log_ID << "Initial conditions:";
    yInfo() << log_ID << "- x_0: "           << eigenToString(x_0);
    yInfo() << log_ID << "- v_0: "           << eigenToString(v_0);
    yInfo() << log_ID << "- euler_0: "       << eigenToString(euler_0);
    yInfo() << log_ID << "- euler_dot_0: "   << eigenToString(euler_0);
    yInfo() << log_ID << "- cov_x_0: "       << eigenToString(cov_x_0);
    yInfo() << log_ID << "- cov_v_0: "       << eigenToString(cov_v_0);
    yInfo() << log_ID << "- cov_eul_0: "     << eigenToString(cov_eul_0);
    yInfo() << log_ID << "- cov_eul_dot_0: " << eigenToString(cov_eul_dot_0);

    yInfo() << log_ID << "Kinematic model:";
    yInfo() << log_ID << "- sample_time:"   << sample_time;
    yInfo() << log_ID << "- psd_acc:"       << eigenToString(kin_psd_acc);
    yInfo() << log_ID << "- psd_euler_acc:" << eigenToString(kin_psd_euler);

    yInfo() << log_ID << "Measurement model:";
    yInfo() << log_ID << "- noise_covariance:" << eigenToString(noise_covariance);

    yInfo() << log_ID << "Unscented transform:";
    yInfo() << log_ID << "- alpha:" << ut_alpha;
    yInfo() << log_ID << "- beta:"  << ut_beta;
    yInfo() << log_ID << "- kappa:" << ut_kappa;

    yInfo() << log_ID << "Point cloud prediction:";
    yInfo() << log_ID << "- num_samples:" << pc_pred_num_samples;

    if (mode == "simulation")
    {
        yInfo() << log_ID << "Simulation:";
        yInfo() << log_ID << "- sample_time:" << sim_sample_time;
        yInfo() << log_ID << "- duration:"    << sim_duration;
        yInfo() << log_ID << "- psd_acc:"     << eigenToString(sim_psd_acc);
        yInfo() << log_ID << "- psd_w:"       << eigenToString(sim_psd_w);
        yInfo() << log_ID << "- x_0:"         << eigenToString(sim_x_0);
        yInfo() << log_ID << "- v_0:"         << eigenToString(sim_v_0);
        yInfo() << log_ID << "- q_0:"         << eigenToString(sim_q_0);
        yInfo() << log_ID << "- w_0:"         << eigenToString(sim_w_0);

        yInfo() << log_ID << "Simulated point cloud:";
        yInfo() << log_ID << "- number_points:"   << sim_point_cloud_number;
        yInfo() << log_ID << "- observer_origin:" << eigenToString(sim_point_cloud_observer);
        yInfo() << log_ID << "- noise_std:"       << eigenToString(sim_point_cloud_noise_std);
        yInfo() << log_ID << "- back_culling:"    << sim_point_cloud_back_culling;
    }

    return 0;
}

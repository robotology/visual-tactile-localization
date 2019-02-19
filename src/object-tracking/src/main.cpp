#include <BoundingBoxEstimator.h>
#include <Correction.h>
#include <Filter.h>
#include <GaussianFilter_.h>
#include <iCubArmModel.h>
#include <iCubHandContactsModel.h>
#include <iCubHandOcclusion.h>
#include <iCubPointCloud.h>
#include <iCubSpringyFingersDetection.h>
#include <InitParticles.h>
#include <DiscreteKinematicModel.h>
#include <DiscretizedKinematicModel.h>
#include <NanoflannPointCloudPrediction.h>
#include <ParticlesCorrection.h>
#include <PFilter.h>
#include <ProximityLikelihood.h>
#include <Random3DPose.h>
#include <SimulatedFilter.h>
#include <SimulatedPointCloud.h>

#include <BayesFilters/AdditiveMeasurementModel.h>
#include <BayesFilters/FilteringAlgorithm.h>
#include <BayesFilters/GaussianCorrection.h>
// #include <BayesFilters/GaussianFilter.h>
#include <BayesFilters/GaussianPrediction.h>
#include <BayesFilters/GPFPrediction.h>
#include <BayesFilters/KFPrediction.h>
#include <BayesFilters/Resampling.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>

#include <Eigen/Dense>

#include <cstdlib>
#include <string>
#include <sstream>

using namespace bfl;
using namespace Eigen;
using namespace yarp::os;

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


std::vector<std::string> loadListString
(
    ResourceFinder &rf,
    const std::string key
)
{
    bool ok = true;

    if (rf.find(key).isNull())
        ok = false;

    Bottle* b = rf.find(key).asList();
    if (b == nullptr)
        ok = false;

    if (!ok)
    {
        yError() << "[Main]" << "Unable to load list of strings with key" << key;
        std::exit(EXIT_FAILURE);
    }

    std::vector<std::string> list;
    for (std::size_t i = 0; i < b->size(); i++)
    {
        Value item_v = b->get(i);
        if (item_v.isNull())
            return std::vector<std::string>();

        if (!item_v.isString())
            return std::vector<std::string>();

        list.push_back(item_v.asString());
    }

    return list;
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

    const std::string port_prefix = "object-tracking";

    ResourceFinder rf;
    rf.setVerbose();
    rf.setDefaultContext("object-tracking");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc, argv);

    /* Get mode of operation. */
    ResourceFinder rf_mode_parameters = rf.findNestedResourceFinder("MODE");
    const std::string mode        = rf_mode_parameters.check("mode",  Value("simulation")).asString();
    const std::string robot       = rf_mode_parameters.check("robot", Value("icubSim")).asString();
    const std::string filter_type = rf_mode_parameters.check("filter_type", Value("ukf")).asString();

    std::unique_ptr<Network> yarp;
    if (mode != "simulation")
    {
        yarp = std::move(std::unique_ptr<Network>(new Network()));

        if (!yarp->checkNetwork())
        {
            yError() << log_ID << "YARP seems unavailable!";
            return EXIT_FAILURE;
        }
    }

    std::size_t number_particles;
    std::size_t eff_number_particles;
    double likelihood_variance;
    std::string point_estimate_method;
    std::size_t point_estimate_window_size;
    double resampling_threshold;
    if (filter_type == "upf")
    {
        /* Get number of particles. */
        ResourceFinder rf_particles = rf.findNestedResourceFinder("PARTICLES");
        number_particles     = rf_particles.check("number",  Value(1)).asInt();
        resampling_threshold = rf_particles.check("resample_threshold", Value(0.5)).asDouble();

        /* Get likelihood variance */
        ResourceFinder rf_likelihood = rf.findNestedResourceFinder("LIKELIHOOD");
        likelihood_variance = rf_likelihood.check("variance",  Value(0.1)).asDouble();

        /* Get point estimate extraction method and window size. */
        ResourceFinder rf_point_estimate = rf.findNestedResourceFinder("POINT_ESTIMATE");
        point_estimate_method      = rf_point_estimate.check("method", Value("smean")).asString();
        point_estimate_window_size = rf_point_estimate.check("window_size", Value(10)).asInt();
    }
    else
        number_particles = 1;

    /* Get initial condition. */
    ResourceFinder rf_initial_conditions = rf.findNestedResourceFinder("INITIAL_CONDITION");
    VectorXd x_0           = loadVectorDouble(rf_initial_conditions, "x_0",           3);
    VectorXd v_0           = loadVectorDouble(rf_initial_conditions, "v_0",           3);
    VectorXd euler_0       = loadVectorDouble(rf_initial_conditions, "euler_0",       3);
    VectorXd euler_dot_0   = loadVectorDouble(rf_initial_conditions, "euler_dot_0",   3);
    VectorXd cov_x_0       = loadVectorDouble(rf_initial_conditions, "cov_x_0",       3);
    VectorXd cov_v_0       = loadVectorDouble(rf_initial_conditions, "cov_v_0",       3);
    VectorXd cov_eul_0     = loadVectorDouble(rf_initial_conditions, "cov_eul_0",     3);
    VectorXd cov_eul_dot_0 = loadVectorDouble(rf_initial_conditions, "cov_eul_dot_0", 3);
    VectorXd center_0      = loadVectorDouble(rf_initial_conditions, "center_0",      6);
    VectorXd radius_0      = loadVectorDouble(rf_initial_conditions, "radius_0",      6);

    /* Kinematic model. */
    ResourceFinder rf_kinematic_model = rf.findNestedResourceFinder("KINEMATIC_MODEL");
    // double sample_time     = rf_kinematic_model.check("sample_time", Value("1.0")).asDouble();
    VectorXd kin_q_x       = loadVectorDouble(rf_kinematic_model, "q_x", 3);
    VectorXd kin_q_x_dot   = loadVectorDouble(rf_kinematic_model, "q_x_dot", 3);
    VectorXd kin_q_eul     = loadVectorDouble(rf_kinematic_model, "q_eul", 3);
    VectorXd kin_q_eul_dot = loadVectorDouble(rf_kinematic_model, "q_eul_dot", 3);

    /* Measurement model. */
    ResourceFinder rf_measurement_model = rf.findNestedResourceFinder("MEASUREMENT_MODEL");
    VectorXd noise_covariance = loadVectorDouble(rf_measurement_model, "noise_covariance", 3);
    MatrixXd noise_covariance_diagonal = noise_covariance.asDiagonal();
    VectorXd tactile_covariance = loadVectorDouble(rf_measurement_model, "tactile_covariance", 3);
    MatrixXd tactile_covariance_diagonal = tactile_covariance.asDiagonal();

    /* Unscented transform. */
    ResourceFinder rf_unscented_transform = rf.findNestedResourceFinder("UNSCENTED_TRANSFORM");
    double ut_alpha = rf_unscented_transform.check("alpha", Value("1.0")).asDouble();
    double ut_beta  = rf_unscented_transform.check("beta", Value("2.0")).asDouble();
    double ut_kappa = rf_unscented_transform.check("kappa", Value("0.0")).asDouble();

    /* Point cloud prediction. */
    ResourceFinder rf_point_cloud_prediction = rf.findNestedResourceFinder("POINT_CLOUD_PREDICTION");
    std::size_t pc_pred_num_samples = rf_point_cloud_prediction.check("number_samples", Value("100")).asInt();

    /* Point cloud filtering. */
    double pc_outlier_threshold;
    if (mode != "simulation")
    {
        ResourceFinder rf_point_cloud_filtering = rf.findNestedResourceFinder("POINT_CLOUD_FILTERING");
        pc_outlier_threshold = rf_point_cloud_filtering.check("outlier_threshold", Value("0.1")).asDouble();
    }

    /* Depth. */
    ResourceFinder rf_depth = rf.findNestedResourceFinder("DEPTH");
    std::string depth_fetch_mode = rf_depth.check("fetch_mode", Value("new_image")).toString();
    std::size_t depth_u_stride = rf_depth.check("u_stride", Value(1)).asInt();
    std::size_t depth_v_stride = rf_depth.check("v_stride", Value(1)).asInt();

    /* Hand occlusion. */
    ResourceFinder rf_hand_occlusion = rf.findNestedResourceFinder("HAND_OCCLUSION");
    bool handle_hand_occlusion            = rf_hand_occlusion.check("handle_occlusion", Value(false)).asBool();
    std::string hand_laterality_occlusion = rf_hand_occlusion.check("laterality", Value("right")).asString();
    double hand_occlusion_scale           = rf_hand_occlusion.check("hull_scale", Value(1.0)).asDouble();

    /* Hand contacts. */
    ResourceFinder rf_hand_contacts = rf.findNestedResourceFinder("HAND_CONTACTS");
    bool handle_hand_contacts                      = rf_hand_contacts.check("handle_contacts", Value(false)).asBool();
    std::string hand_laterality_contacts           = rf_hand_contacts.check("laterality", Value()).asString();
    std::vector<std::string> used_fingers_contacts = loadListString(rf_hand_contacts, "used_fingers");

    /* Simulation parameters. */
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
        ResourceFinder rf_simulation = rf.findNestedResourceFinder("SIMULATION");
        sim_sample_time = rf_simulation.check("sample_time", Value("1.0")).asDouble();
        sim_duration    = rf_simulation.check("duration", Value("1.0")).asDouble();
        sim_psd_acc     = loadVectorDouble(rf_simulation, "psd_acc", 3);
        sim_psd_w       = loadVectorDouble(rf_simulation, "psd_w", 3);
        sim_x_0         = loadVectorDouble(rf_simulation, "x_0", 3);
        sim_v_0         = loadVectorDouble(rf_simulation, "v_0", 3);
        sim_q_0         = loadVectorDouble(rf_simulation, "q_0", 4);
        sim_w_0         = loadVectorDouble(rf_simulation, "w_0", 3);

        ResourceFinder rf_sim_point_cloud = rf.findNestedResourceFinder("SIMULATED_POINT_CLOUD");
        sim_point_cloud_number       = rf_sim_point_cloud.check("number_points", Value("1000")).asInt();
        sim_point_cloud_observer     = loadVectorDouble(rf_sim_point_cloud, "observer_origin", 3);
        sim_point_cloud_noise_std    = loadVectorDouble(rf_sim_point_cloud, "noise_std", 3);
        sim_point_cloud_back_culling = rf_sim_point_cloud.check("back_cullnig", Value(true)).asBool();
    }

    /* Mesh parameters. */
    ResourceFinder rf_object = rf.findNestedResourceFinder("OBJECT");
    const std::string object_name          = rf_object.check("object_name", Value("ycb_mustard")).asString();
    const std::string object_mesh_path_obj = rf.findPath("mesh/" + object_name) + "/nontextured.obj";
    const std::string object_mesh_path_ply = rf.findPath("mesh/" + object_name) + "/nontextured.ply";

    /* Bounding box estimator. */
    VectorXd bbox_cov_0;
    VectorXd bbox_Q;
    VectorXd bbox_R;
    VectorXd bbox_tl_0;
    VectorXd bbox_br_0;

    ResourceFinder rf_bbox = rf.findNestedResourceFinder("BBOX_ESTIMATOR");
    const std::string iol_object_name = rf_bbox.check("iol_object_name", Value("Bottle")).asString();
    bbox_cov_0                        = loadVectorDouble(rf_bbox, "cov_0", 4);
    bbox_Q                            = loadVectorDouble(rf_bbox, "Q", 4);
    bbox_R                            = loadVectorDouble(rf_bbox, "R", 4);
    bool use_bbox_0                   = rf_bbox.check("use_bbox_0", Value(false)).asBool();
    if (use_bbox_0)
    {
        bbox_tl_0 = loadVectorDouble(rf_bbox, "bbox_tl_0", 2);
        bbox_br_0 = loadVectorDouble(rf_bbox, "bbox_br_0", 2);
    }
    MatrixXd bbox_cov_0_diagonal = bbox_cov_0.asDiagonal();
    MatrixXd bbox_Q_diagonal = bbox_Q.asDiagonal();
    MatrixXd bbox_R_diagonal = bbox_R.asDiagonal();
    double iol_bbox_scale = rf_bbox.check("iol_bbox_scale", Value(1.0)).asDouble();

    /* Logging parameters. */
    ResourceFinder rf_logging = rf.findNestedResourceFinder("LOG");
    bool enable_log = rf_logging.check("enable_log", Value(false)).asBool();
    const std::string log_path = rf_logging.check("absolute_log_path", Value("")).asString();
    if (enable_log && log_path == "")
    {
        yWarning() << "Invalid log path. Disabling log...";
        enable_log = false;
    }

    /* Miscellaneous. */
    bool enable_send_hull;
    bool enable_send_mask;
    if (mode != "SIMULATION")
    {
        ResourceFinder rf_misc = rf.findNestedResourceFinder("MISC");
        enable_send_hull = rf_misc.check("send_hull", Value(false)).asBool();
        enable_send_mask = rf_misc.check("send_mask", Value(false)).asBool();
    }

    /* Log parameters. */
    yInfo() << log_ID << "Mode of operation:";
    yInfo() << log_ID << "- mode:"        << mode;
    yInfo() << log_ID << "- robot:"       << robot;
    yInfo() << log_ID << "- filter_type:" << filter_type;

    if (filter_type == "upf")
    {
        yInfo() << log_ID << "Particles:";
        yInfo() << log_ID << "- number:"             << number_particles;
        yInfo() << log_ID << "- resample_threshold:" << resampling_threshold;

        yInfo() << log_ID << "Likelihood:";
        yInfo() << log_ID << "- variance:" << likelihood_variance;
    }

    yInfo() << log_ID << "Initial conditions:";
    if (filter_type == "ukf")
    {
        yInfo() << log_ID << "- x_0: "           << eigenToString(x_0);
        yInfo() << log_ID << "- v_0: "           << eigenToString(v_0);
        yInfo() << log_ID << "- euler_0: "       << eigenToString(euler_0);
        yInfo() << log_ID << "- euler_dot_0: "   << eigenToString(euler_dot_0);
    }
    else if (filter_type == "upf")
    {
        yInfo() << log_ID << "- center_0: "      << eigenToString(center_0);
        yInfo() << log_ID << "- radius_0: "      << eigenToString(radius_0);
    }
    yInfo() << log_ID << "- cov_x_0: "       << eigenToString(cov_x_0);
    yInfo() << log_ID << "- cov_v_0: "       << eigenToString(cov_v_0);
    yInfo() << log_ID << "- cov_eul_0: "     << eigenToString(cov_eul_0);
    yInfo() << log_ID << "- cov_eul_dot_0: " << eigenToString(cov_eul_dot_0);

    yInfo() << log_ID << "Kinematic model:";
    // yInfo() << log_ID << "- sample_time:" << sample_time;
    yInfo() << log_ID << "- q_x:"         << eigenToString(kin_q_x);
    yInfo() << log_ID << "- q_x_dot:"     << eigenToString(kin_q_x_dot);
    yInfo() << log_ID << "- q_eul:"       << eigenToString(kin_q_eul);
    yInfo() << log_ID << "- q_eul_dot:"   << eigenToString(kin_q_eul_dot);

    yInfo() << log_ID << "Measurement model:";
    yInfo() << log_ID << "- noise_covariance:" << eigenToString(noise_covariance);

    yInfo() << log_ID << "Unscented transform:";
    yInfo() << log_ID << "- alpha:" << ut_alpha;
    yInfo() << log_ID << "- beta:"  << ut_beta;
    yInfo() << log_ID << "- kappa:" << ut_kappa;

    yInfo() << log_ID << "Point cloud prediction:";
    yInfo() << log_ID << "- num_samples:" << pc_pred_num_samples;

    yInfo() << log_ID << "Point cloud filtering:";
    yInfo() << log_ID << "- outlier_threshold:" << pc_outlier_threshold;

    yInfo() << log_ID << "Depth:";
    yInfo() << log_ID << "- fetch_mode:" << depth_fetch_mode;
    yInfo() << log_ID << "- u_stride:" << depth_u_stride;
    yInfo() << log_ID << "- v_stride:" << depth_v_stride;

    yInfo() << log_ID << "Hand occlusion:";
    yInfo() << log_ID << "- handle_occlusion:" << handle_hand_occlusion;
    yInfo() << log_ID << "- hull_scale:" << hand_occlusion_scale;

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

    yInfo() << log_ID << "Object:";
    yInfo() << log_ID << "- object_name:"        << object_name;
    yInfo() << log_ID << "- mesh path is (obj):" << object_mesh_path_obj;
    yInfo() << log_ID << "- mesh path is (ply):" << object_mesh_path_ply;

    yInfo() << log_ID << "Bounding box estimator:";
    yInfo() << log_ID << "- iol_object_name:" << iol_object_name;
    yInfo() << log_ID << "- iol_bbox_scale:"  << iol_bbox_scale;
    yInfo() << log_ID << "- use_bbox_0:"      << use_bbox_0;
    if (use_bbox_0)
    {
        yInfo() << log_ID << "- bbox_tl_0:" << eigenToString(bbox_tl_0);
        yInfo() << log_ID << "- bbox_br_0:" << eigenToString(bbox_br_0);
    }
    yInfo() << log_ID << "- cov_0"            << eigenToString(bbox_cov_0);
    yInfo() << log_ID << "- Q"                << eigenToString(bbox_Q);
    yInfo() << log_ID << "-R"                 << eigenToString(bbox_R);

    yInfo() << log_ID << "Logging:";
    yInfo() << log_ID << "- enable_log:"        << enable_log;
    yInfo() << log_ID << "- absolute_log_path:" << log_path;

    yInfo() << log_ID << "Miscellaneous:";
    yInfo() << log_ID << "- send_hull:" << enable_send_hull;
    yInfo() << log_ID << "- send_mask:" << enable_send_mask;

    /**
     * Initialize point cloud prediction.
     */
    std::unique_ptr<PointCloudPrediction> pc_prediction =
        std::unique_ptr<NanoflannPointCloudPrediction>(new NanoflannPointCloudPrediction(object_mesh_path_ply, pc_pred_num_samples));

    /**
     * Initialize measurement model.
     */
    std::unique_ptr<AdditiveMeasurementModel> measurement_model;
    std::shared_ptr<iCubPointCloudExogenousData> icub_pc_shared_data = std::make_shared<iCubPointCloudExogenousData>();
    if (mode == "simulation")
    {
        /**
         * Initial condition of simulation.
         */
        VectorXd state_0(3 + 3 + 4 + 3);
        state_0.head(3) = sim_x_0;
        state_0.segment(3, 3) = sim_v_0;
        state_0.segment(6, 4) = sim_q_0;
        state_0.tail(3) = sim_w_0;

        /**
         * Initialize random pose generator.
         */
        std::unique_ptr<StateModel> rand_pose(
            new Random3DPose(sim_sample_time,
                             sim_psd_acc(0), sim_psd_acc(1), sim_psd_acc(2),
                             sim_psd_w(0), sim_psd_w(1), sim_psd_w(2)));

        /**
         * Initialize simulated state model.
         */
        std::unique_ptr<SimulatedStateModel> sim_rand_pose(
            new SimulatedStateModel(std::move(rand_pose), state_0, sim_duration / sim_sample_time));
        if (enable_log)
            sim_rand_pose->enable_log(log_path, "object-tracking");

        /**
         * Initialize simulated measurement model.
         */
        std::unique_ptr<SimulatedPointCloud> pc_simulation =
            std::unique_ptr<SimulatedPointCloud>(new SimulatedPointCloud(object_mesh_path_ply,
                                                                         std::move(pc_prediction),
                                                                         std::move(sim_rand_pose),
                                                                         // This is the modeled noise covariance
                                                                         noise_covariance_diagonal,
                                                                         sim_point_cloud_observer,
                                                                         sim_point_cloud_number,
                                                                         sim_point_cloud_back_culling));
        // This the actual simulated noise standard deviation
        pc_simulation->enableNoise(sim_point_cloud_noise_std(0),
                                   sim_point_cloud_noise_std(1),
                                   sim_point_cloud_noise_std(2));

        // Prefetch measurements
        std::cout << "Prefetching measurements for faster simulation..." << std::flush;
        pc_simulation->prefetchMeasurements(sim_duration / sim_sample_time);
        std::cout << "done." << std::endl;

        measurement_model = std::move(pc_simulation);
    }
    else
    {
        std::unique_ptr<iCubPointCloud> pc_icub = std::unique_ptr<iCubPointCloud>(
            new iCubPointCloud(std::move(pc_prediction),
                               noise_covariance_diagonal,
                               tactile_covariance_diagonal,
                               port_prefix,
                               "left",
                               depth_fetch_mode,
                               pc_outlier_threshold,
                               depth_u_stride,
                               depth_v_stride,
                               enable_send_hull,
                               icub_pc_shared_data));

        if (handle_hand_occlusion)
        {
            /* Initialize iCubArmModel providing the 3D pose of the hand parts relative to the hand palm. */
            std::unique_ptr<iCubArmModel> icub_arm = std::unique_ptr<iCubArmModel>(
                new iCubArmModel(true,
                                 false,
                                 hand_laterality_occlusion,
                                 "object-tracking",
                                 "object-tracking/icub-arm-model/occlusion/" + hand_laterality_occlusion));

            /* Initialize iCubHandOcclusion that reads the hand palm pose from a port
               and creates an occlusion mask to be used to clean part of the point cloud of the object
               from undesired parts due to hand occlusion. */
            std::unique_ptr<iCubHandOcclusion> hand_occlusion = std::unique_ptr<iCubHandOcclusion>(
                new iCubHandOcclusion(std::move(icub_arm),
                                      "object-tracking/icub-hand-occlusion/" + hand_laterality_occlusion,
                                      "left",
                                      hand_occlusion_scale));

            /* Add the occlusion to the iCubPointCloud. */
            pc_icub->addObjectOcclusion(std::move(hand_occlusion));
        }

        if (handle_hand_contacts)
        {
            /* Initialize iCubArmModel providing the 3D pose of the hand parts relative to the hand palm. */ 
            std::unique_ptr<iCubArmModel> icub_arm = std::unique_ptr<iCubArmModel>(
                new iCubArmModel(false,
                                 false,
                                 hand_laterality_contacts,
                                 "object-tracking",
                                 "object-tracking/icub-arm-model/contacts/" + hand_laterality_contacts));

            std::unique_ptr<iCubSpringyFingersDetection> icub_springy_fingers = std::unique_ptr<iCubSpringyFingersDetection>(
                new iCubSpringyFingersDetection("right"));

            std::unique_ptr<iCubHandContactsModel> icub_contacts = std::unique_ptr<iCubHandContactsModel>(
                new iCubHandContactsModel(std::move(icub_arm),
                                          std::move(icub_springy_fingers),
                                          used_fingers_contacts,
                                          "object-tracking/icub-hand-contacts"));

            /* Add the contacts to the iCubPointCloud. */
            pc_icub->addObjectContacts(std::move(icub_contacts));
        }

        measurement_model = std::move(pc_icub);
    }

    if (enable_log)
        measurement_model->enable_log(log_path, "object-tracking");

    /**
     * Filter construction.
     */

    /**
     * StateModel
     */
    // std::unique_ptr<LinearStateModel> kinematic_model = std::unique_ptr<DiscreteKinematicModel>(
    //     new DiscreteKinematicModel(sample_time,
    //                                kin_q_x(0), kin_q_x(1), kin_q_x(2),
    //                                kin_q_x_dot(0), kin_q_x_dot(1), kin_q_x_dot(2),
    //                                kin_q_eul(0), kin_q_eul(1), kin_q_eul(2),
    //                                kin_q_eul_dot(0), kin_q_eul_dot(1), kin_q_eul_dot(2)));
    std::unique_ptr<LinearStateModel> kinematic_model = std::unique_ptr<DiscretizedKinematicModel>(
        new DiscretizedKinematicModel(// sample_time,
                                      kin_q_x(0), kin_q_x(1), kin_q_x(2),
                                      kin_q_eul(0), kin_q_eul(1), kin_q_eul(2)));

    std::size_t dim_linear;
    std::size_t dim_circular;
    std::tie(dim_linear, dim_circular) = kinematic_model->getOutputSize();
    std::size_t state_size = dim_linear + dim_circular;

    /**
     * Initial condition.
     */

    /* Used in both UKF and UPF. */
    VectorXd initial_covariance(12);
    initial_covariance.head<3>() = cov_x_0;
    initial_covariance.segment<3>(3) = cov_v_0;
    initial_covariance.segment<3>(6) = cov_eul_dot_0;
    initial_covariance.tail<3>() = cov_eul_0;

    /* Used only in the UKF. */
    Gaussian initial_state(dim_linear, dim_circular);
    VectorXd initial_mean(12);
    initial_mean.head<3>() = x_0;
    initial_mean.segment<3>(3) = v_0;
    initial_mean.segment<3>(6) = euler_dot_0;
    initial_mean.tail<3>() = euler_0;

    initial_state.mean() =  initial_mean;
    initial_state.covariance() = initial_covariance.asDiagonal();

    /**
     * Prediction step.
     */
    std::unique_ptr<GaussianPrediction> prediction =
        std::unique_ptr<KFPrediction>(new KFPrediction(std::move(kinematic_model)));

    /**
     * Correction step.
     */
    std::size_t measurement_sub_size = 3; // i.e. a measurement is made of 3 * N points
                                          // where N is the number of points belonging to the point cloud

    std::unique_ptr<Correction> correction =
        std::unique_ptr<Correction>(new Correction(std::move(measurement_model),
                                                   state_size,
                                                   ut_alpha, ut_beta, ut_kappa,
                                                   measurement_sub_size));

    /**
     * BoundingBoxEstimator initialization.
     */
    std::unique_ptr<BoundingBoxEstimator> bbox_estimator;

    if (use_bbox_0)
    {
        // Giving the initial bounding box of the object from outside
        std::pair<int, int> top_left = std::make_pair(static_cast<int>(bbox_tl_0(0)), static_cast<int>(bbox_tl_0(1)));
        std::pair<int, int> bottom_right = std::make_pair(static_cast<int>(bbox_br_0(0)), static_cast<int>(bbox_br_0(1)));
        bbox_estimator = std::unique_ptr<BoundingBoxEstimator>(
            new BoundingBoxEstimator(std::make_pair(top_left, bottom_right),
                                     number_particles,
                                     "object-tracking/bbox-estimator",
                                     "left",
                                     object_mesh_path_obj,
                                     rf.findPath("shader/"),
                                     iol_object_name,
                                     iol_bbox_scale,
                                     enable_send_mask,
                                     bbox_cov_0_diagonal,
                                     bbox_Q_diagonal,
                                     bbox_R_diagonal));
    }
    else
    {
        bbox_estimator = std::unique_ptr<BoundingBoxEstimator>(
            new BoundingBoxEstimator(number_particles,
                                     "object-tracking/bbox-estimator",
                                     "left",
                                     object_mesh_path_obj,
                                     rf.findPath("shader/"),
                                     iol_object_name,
                                     iol_bbox_scale,
                                     enable_send_mask,
                                     bbox_cov_0_diagonal,
                                     bbox_Q_diagonal,
                                     bbox_R_diagonal));
    }

    /**
     * The maximum allowed number of particles depend on the sicad engine within the BoundingBoxEstimator
     */
    eff_number_particles = bbox_estimator->getNumberComponents();
    yInfo() << log_ID << "Requested" << number_particles << "particles, allowed" << eff_number_particles << "particles.";

    /**
     * Filter.
     */
    std::cout << "Initializing filter..." << std::flush;

    std::unique_ptr<FilteringAlgorithm> filter;

    if (filter_type == "ukf")
    {
        if (mode == "simulation")
        {
            filter = std::move(std::unique_ptr<SimulatedFilter>(
                                   new SimulatedFilter(initial_state,
                                                       std::move(prediction),
                                                       std::move(correction),
                                                       sim_duration / sim_sample_time)));
        }
        else if ((mode == "robot" || mode == "play"))
        {
            filter = std::move(std::unique_ptr<Filter>(
                                   new Filter(port_prefix,
                                              initial_state,
                                              std::move(prediction),
                                              std::move(correction),
                                              std::move(bbox_estimator),
                                              icub_pc_shared_data)));
        }
    }
    else if (filter_type == "upf")
    {
        /* Particles initialization. */
        MatrixXd covariance_0 = initial_covariance.asDiagonal();
        std::unique_ptr<InitParticles> pf_initialization = std::unique_ptr<InitParticles>(
            new InitParticles(center_0, radius_0, covariance_0));

        /* Gaussian particle filter perdiction. */
        std::unique_ptr<GPFPrediction> pf_prediction = std::unique_ptr<GPFPrediction>(
            new GPFPrediction(std::move(prediction)));

        /* Gaussian particle filter correction. */
        std::unique_ptr<ParticlesCorrection> pf_correction;

        /* Likelihood. */
        std::unique_ptr<NanoflannPointCloudPrediction> distances_approximation = std::unique_ptr<NanoflannPointCloudPrediction>(
            new NanoflannPointCloudPrediction(object_mesh_path_ply, pc_pred_num_samples));

        std::unique_ptr<ProximityLikelihood> proximity_likelihood = std::unique_ptr<ProximityLikelihood>(
            new ProximityLikelihood(likelihood_variance, std::move(distances_approximation)));

        pf_correction = std::unique_ptr<ParticlesCorrection>(
            new ParticlesCorrection(std::move(correction), std::move(proximity_likelihood)));

        /* Resampling. */
        std::unique_ptr<Resampling> pf_resampling = std::unique_ptr<Resampling>(new Resampling());

        if (mode == "simulation")
        {
            // TO BE DONE
        }
        else if ((mode == "robot" || mode == "play"))
        {
            filter = std::move(std::unique_ptr<PFilter>(
                                   new PFilter(port_prefix,
                                               eff_number_particles,
                                               resampling_threshold,
                                               point_estimate_method,
                                               point_estimate_window_size,
                                               std::move(pf_initialization),
                                               std::move(pf_prediction),
                                               std::move(pf_correction),
                                               std::move(pf_resampling),
                                               std::move(bbox_estimator),
                                               icub_pc_shared_data)));
        }
    }

    std::cout << "done." << std::endl;

    if (enable_log)
        filter->enable_log(log_path, "object-tracking");

    std::cout << "Booting filter..." << std::flush;

    filter->boot();

    std::cout << "done." << std::endl;

    if (mode == "simulation")
    {
        std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
        filter->run();

        std::cout << "Running filter..." << std::endl;

        if (!filter->wait())
            return EXIT_FAILURE;

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

        std::cout << "Done in "
                  << std::chrono::duration_cast<std::chrono::seconds>(end - start).count()
                  << "s"
                  << std::endl;
    }
    else
    {
        std::cout << "Running filter..." << std::endl;

        if (!filter->wait())
            return EXIT_FAILURE;
    }

    yInfo() << log_ID << "Application closed succesfully.";

    return EXIT_SUCCESS;
}

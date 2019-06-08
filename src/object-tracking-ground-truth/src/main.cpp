/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ArucoMeasurement.h>
#include <ArucoTracker.h>
#include <Camera.h>
#include <Correction.h>
#include <iCubCamera.h>
#include <KinematicModel.h>
#include <RealsenseCamera.h>

#include <BayesFilters/AdditiveMeasurementModel.h>
#include <BayesFilters/Gaussian.h>
#include <BayesFilters/KFPrediction.h>
#include <BayesFilters/LinearStateModel.h>
#include <BayesFilters/LinearMeasurementModel.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>

#include <cstdlib>
#include <string>

// using namespace bfl;
using namespace Eigen;
using namespace bfl;
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


VectorXi loadVectorInt
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
        yError() << "[Main]" << "Unable to load vector" << key;
        std::exit(EXIT_FAILURE);
    }

    VectorXi vector(b->size());
    for (std::size_t i = 0; i < b->size(); i++)
    {
        Value item_v = b->get(i);
        if (item_v.isNull())
            return VectorXi(0);

        if (!item_v.isInt())
            return VectorXi(0);

        vector(i) = item_v.asInt();
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

    const std::string port_prefix = "object-tracking-ground-truth";

    std::unique_ptr<Network> yarp;
    yarp = std::move(std::unique_ptr<Network>(new Network()));

    if (!yarp->checkNetwork())
    {
            yError() << log_ID << "YARP seems unavailable!";
            return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.setVerbose();
    rf.setDefaultContext("object-tracking-ground-truth");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc, argv);

    ResourceFinder rf_object_tracking;
    rf_object_tracking.setVerbose(true);
    rf_object_tracking.setDefaultContext("object-tracking");
    rf_object_tracking.setDefaultConfigFile("config.ini");
    rf_object_tracking.configure(0, NULL);

    /* Get object name. */
    ResourceFinder rf_object = rf_object_tracking.findNestedResourceFinder("OBJECT");
    const std::string object_name = rf_object.check("object_name", Value("ycb_mustard_bottle")).asString();

    /* Get camera name. */
    ResourceFinder rf_camera = rf.findNestedResourceFinder("CAMERA");
    const std::string camera_name = rf_camera.check("camera", Value("iCubCamera")).asString();
    const std::string camera_laterality = rf_camera.check("camera_laterality", Value("")).asString();
    const std::string camera_fallback_context = rf_camera.check("fallback_context", Value("object-tracking-ground-truth")).asString();
    const std::string camera_fallback_key = rf_camera.check("fallback_config", Value("left_320_240")).asString();

    /* Get marker properties. */
    ResourceFinder rf_offsets;
    rf_offsets.setVerbose();
    rf_offsets.setDefaultContext("object-tracking-ground-truth");
    rf_offsets.setDefaultConfigFile("marker_offsets.ini");
    rf_offsets.configure(argc, argv);
    ResourceFinder rf_offsets_object = rf_offsets.findNestedResourceFinder(object_name.c_str());

    VectorXi marker_ids = loadVectorInt(rf_offsets_object, "marker_ids");

    VectorXd marker_lengths = loadVectorDouble(rf_offsets_object, "marker_sizes", marker_ids.size());

    std::vector<VectorXd> marker_offsets;
    for (std::size_t i = 0; i < marker_ids.size(); i++)
        marker_offsets.push_back(loadVectorDouble(rf_offsets_object, "offset" + std::to_string(i), 6));

    /* Get length of ArUco marker side. */
    ResourceFinder rf_aruco = rf.findNestedResourceFinder("ARUCO");
    double marker_side_length = rf_aruco.check("side_length", Value(0.01)).asDouble();

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

    /* Kinematic model. */
    ResourceFinder rf_kinematic_model = rf.findNestedResourceFinder("KINEMATIC_MODEL");
    VectorXd kin_q_x       = loadVectorDouble(rf_kinematic_model, "q_x", 3);
    VectorXd kin_q_eul     = loadVectorDouble(rf_kinematic_model, "q_eul", 3);

    /* Measurement model. */
    ResourceFinder rf_measurement_model = rf.findNestedResourceFinder("MEASUREMENT_MODEL");
    VectorXd noise_covariance = loadVectorDouble(rf_measurement_model, "noise_covariance", 6);
    MatrixXd noise_covariance_diagonal = noise_covariance.asDiagonal();

    /* Logging parameters. */
    ResourceFinder rf_logging = rf.findNestedResourceFinder("LOG");
    bool enable_log = rf_logging.check("enable_log", Value(false)).asBool();
    const std::string log_path = rf_logging.check("absolute_log_path", Value("")).asString();
    if (enable_log && log_path == "")
    {
        yWarning() << "Invalid log path. Disabling log...";
        enable_log = false;
    }

    /* Miscellaneous parameters. */
    ResourceFinder rf_misc = rf.findNestedResourceFinder("MISC");
    bool send_aruco_image    = rf_misc.check("send_image", Value(false)).asBool();
    bool send_aruco_estimate = rf_misc.check("send_aruco_estimate", Value(false)).asBool();

    /* Log parameters. */
    yInfo() << log_ID << "Camera:";
    yInfo() << log_ID << "- camera:" << camera_name;
    yInfo() << log_ID << "- fallback_context" << camera_fallback_context;
    yInfo() << log_ID << "- fallback_config" << camera_fallback_key;

    yInfo() << log_ID << "Initial conditions:";
    yInfo() << log_ID << "- x_0: "           << eigenToString(x_0);
    yInfo() << log_ID << "- v_0: "           << eigenToString(v_0);
    yInfo() << log_ID << "- euler_0: "       << eigenToString(euler_0);
    yInfo() << log_ID << "- euler_dot_0: "   << eigenToString(euler_dot_0);
    yInfo() << log_ID << "- cov_x_0: "       << eigenToString(cov_x_0);
    yInfo() << log_ID << "- cov_v_0: "       << eigenToString(cov_v_0);
    yInfo() << log_ID << "- cov_eul_0: "     << eigenToString(cov_eul_0);
    yInfo() << log_ID << "- cov_eul_dot_0: " << eigenToString(cov_eul_dot_0);

    yInfo() << log_ID << "Kinematic model:";
    yInfo() << log_ID << "- q_x:"         << eigenToString(kin_q_x);
    yInfo() << log_ID << "- q_eul:"       << eigenToString(kin_q_eul);

    yInfo() << log_ID << "Measurement model:";
    yInfo() << log_ID << "- noise_covariance:" << eigenToString(noise_covariance);

    yInfo() << log_ID << "Object:";
    yInfo() << log_ID << "- object_name:"   << object_name;

    yInfo() << log_ID << "Aruco:";
    for (std::size_t i = 0; i < marker_ids.size(); i++)
    {
        yInfo() << log_ID << "- marker_" << marker_ids(i) <<":";
        yInfo() << log_ID << "__length:" << marker_lengths(i);
        yInfo() << log_ID << "__offset:" << eigenToString(marker_offsets.at(i));
    }

    yInfo() << log_ID << "Logging:";
    yInfo() << log_ID << "- enable_log:"        << enable_log;
    yInfo() << log_ID << "- absolute_log_path:" << log_path;

    yInfo() << log_ID << "Misc:";
    yInfo() << log_ID << "- send_aruco_image:"    << send_aruco_image;
    yInfo() << log_ID << "- send_aruco_estimate:" << send_aruco_estimate;

    /**
     * Filter construction.
     */

    /**
     * StateModel
     */
    std::unique_ptr<LinearStateModel> kinematic_model = std::unique_ptr<KinematicModel>(
        new KinematicModel(kin_q_x(0), kin_q_x(1), kin_q_x(2),
                           kin_q_eul(0), kin_q_eul(1), kin_q_eul(2)));

    /**
     * Camera.
     */
    std::unique_ptr<Camera> camera;
    if (camera_name == "iCubCamera")
    {
        camera = std::unique_ptr<iCubCamera>
        (
            new iCubCamera(camera_laterality, port_prefix, camera_fallback_context, camera_fallback_key)
        );
    }
    else if (camera_name == "RealsenseCamera")
    {
        camera = std::unique_ptr<RealsenseCamera>
        (
            new RealsenseCamera(port_prefix, camera_fallback_context, camera_fallback_key)
        );
    }
    else
    {
        yError() << log_ID << "The requested camera is not available. Requested camera is" << camera_name;
        std::exit(EXIT_FAILURE);
    }
    camera->initialize();

    /**
     * ArUco measurement model.
     */
    std::unique_ptr<LinearMeasurementModel> measurement_model = std::unique_ptr<LinearMeasurementModel>(
        new ArucoMeasurement(std::move(camera), port_prefix, marker_ids, marker_lengths, marker_offsets, noise_covariance_diagonal, send_aruco_image, send_aruco_estimate));

    /**
     * Initial condition.
     */
    std::size_t dim_linear;
    std::size_t dim_circular;
    std::tie(dim_linear, dim_circular) = kinematic_model->getOutputSize();
    std::size_t state_size = dim_linear + dim_circular;

    VectorXd initial_mean(12);
    initial_mean.head<3>() = x_0;
    initial_mean.segment<3>(3) = v_0;
    initial_mean.segment<3>(6) = euler_dot_0;
    initial_mean.tail<3>() = euler_0;

    VectorXd initial_covariance(12);
    initial_covariance.head<3>() = cov_x_0;
    initial_covariance.segment<3>(3) = cov_v_0;
    initial_covariance.segment<3>(6) = cov_eul_dot_0;
    initial_covariance.tail<3>() = cov_eul_0;

    Gaussian initial_state(dim_linear, dim_circular);
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
    std::unique_ptr<Correction> correction =
        std::unique_ptr<Correction>(new Correction(std::move(measurement_model)));

    /**
     * Filter.
     */
    std::cout << "Initializing filter..." << std::flush;

    std::unique_ptr<FilteringAlgorithm> filter = std::unique_ptr<ArucoTracker>(
        new ArucoTracker(port_prefix,
                         initial_state,
                         std::move(prediction),
                         std::move(correction)));

    std::cout << "done." << std::endl;

    if (enable_log)
        filter->enable_log(log_path, "object-tracking-ground-truth");

    std::cout << "Booting filter..." << std::flush;

    filter->boot();

    std::cout << "done." << std::endl;

    std::cout << "Running filter..." << std::endl;

    if (!filter->wait())
        return EXIT_FAILURE;

    yInfo() << log_ID << "Application closed succesfully.";

    return EXIT_SUCCESS;
}

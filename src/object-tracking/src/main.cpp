/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <BoundingBoxSegmentation.h>
#include <Camera.h>
#include <Correction.h>
#include <iCubArmModel.h>
#include <iCubCamera.h>
#include <iCubHandContactsModel.h>
#include <iCubHandOcclusion.h>
#include <iCubObjectMeasurements.h>
#include <iCubSpringyFingersDetection.h>
#include <InHandObjectSegmentation.h>
#include <InitParticles.h>
#include <DiscretizedKinematicModel.h>
#include <DiscretizedKinematicModelTDD.h>
#include <MaskSegmentation.h>
#include <NanoflannPointCloudPrediction.h>
#include <ObjectRenderer.h>
#include <ObjectSampler.h>
#include <ObjectMeshSampler.h>
#include <ParticlesCorrection.h>
#include <PointCloudSegmentation.h>
#include <PFilter.h>
#include <ProximityLikelihood.h>
#include <RealsenseCamera.h>
#ifdef USE_SUPERQUADRICLIB
#include <SuperquadricSampler.h>
#endif

#include <BayesFilters/AdditiveMeasurementModel.h>
#include <BayesFilters/FilteringAlgorithm.h>
#include <BayesFilters/GaussianCorrection.h>
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
#ifdef USE_SUPERQUADRICLIB
using namespace SuperqModel;
#endif

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


std::string eigenToString(const Ref<const VectorXd>& v)
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
    rf.configure(argc, argv);

    /* Get robot name. */
    const std::string robot = rf.check("robot", Value("icub")).asString();

    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError() << log_ID << "YARP seems unavailable!";
        return EXIT_FAILURE;
    }

    /* Get number of particles. */
    ResourceFinder rf_particles = rf.findNestedResourceFinder("PARTICLES");
    const std::size_t number_particles = rf_particles.check("number",  Value(1)).asInt();
    const double resampling_threshold  = rf_particles.check("resample_threshold", Value(0.5)).asDouble();

    /* Get likelihood variance */
    ResourceFinder rf_likelihood = rf.findNestedResourceFinder("LIKELIHOOD");
    const double likelihood_variance = rf_likelihood.check("variance",  Value(0.1)).asDouble();

    /* Get point estimate extraction method and window size. */
    ResourceFinder rf_point_estimate = rf.findNestedResourceFinder("POINT_ESTIMATE");
    const std::string point_estimate_method      = rf_point_estimate.check("method", Value("smean")).asString();
    const std::size_t point_estimate_window_size = rf_point_estimate.check("window_size", Value(10)).asInt();

    /* Get initial conditions. */
    ResourceFinder rf_initial_conditions = rf.findNestedResourceFinder("INITIAL_CONDITION");
    VectorXd cov_x_0       = loadVectorDouble(rf_initial_conditions, "cov_x_0",       3);
    VectorXd cov_v_0       = loadVectorDouble(rf_initial_conditions, "cov_v_0",       3);
    VectorXd cov_eul_0     = loadVectorDouble(rf_initial_conditions, "cov_eul_0",     3);
    VectorXd cov_eul_dot_0 = loadVectorDouble(rf_initial_conditions, "cov_eul_dot_0", 3);
    VectorXd center_0      = loadVectorDouble(rf_initial_conditions, "center_0",      6);
    VectorXd radius_0      = loadVectorDouble(rf_initial_conditions, "radius_0",      6);

    /* Camera parameters. */
    ResourceFinder rf_camera = rf.findNestedResourceFinder("CAMERA");
    const std::string camera_name         = rf_camera.check("name", Value("iCubCamera")).asString();
    const std::string camera_laterality   = rf_camera.check("laterality", Value("left")).asString();
    const std::string camera_fallback_key = rf_camera.check("fallback_key", Value("icub_320_240")).asString();

    /* Kinematic model. */
    ResourceFinder rf_kinematic_model = rf.findNestedResourceFinder("KINEMATIC_MODEL");
    VectorXd kin_q_x             = loadVectorDouble(rf_kinematic_model, "q_x", 3);
    VectorXd kin_q_eul           = loadVectorDouble(rf_kinematic_model, "q_eul", 3);
    const bool use_tdd_occlusion = rf_kinematic_model.check("use_tdd_occlusion", Value(false)).asBool();
    const double tdd_max_seconds = rf_kinematic_model.check("tdd_max_seconds", Value(10.0)).asDouble();
    const double tdd_exp_gain    = rf_kinematic_model.check("tdd_exp_gain", Value(1.0)).asDouble();

    /* Measurement model. */
    ResourceFinder rf_measurement_model = rf.findNestedResourceFinder("MEASUREMENT_MODEL");
    VectorXd visual_covariance           = loadVectorDouble(rf_measurement_model, "visual_covariance", 3);
    MatrixXd visual_covariance_diagonal  = visual_covariance.asDiagonal();
    VectorXd tactile_covariance          = loadVectorDouble(rf_measurement_model, "tactile_covariance", 3);
    MatrixXd tactile_covariance_diagonal = tactile_covariance.asDiagonal();

    /* Unscented transform. */
    ResourceFinder rf_unscented_transform = rf.findNestedResourceFinder("UNSCENTED_TRANSFORM");
    const double ut_alpha = rf_unscented_transform.check("alpha", Value("1.0")).asDouble();
    const double ut_beta  = rf_unscented_transform.check("beta", Value("2.0")).asDouble();
    const double ut_kappa = rf_unscented_transform.check("kappa", Value("0.0")).asDouble();

    /* Point cloud prediction. */
    ResourceFinder rf_point_cloud_prediction = rf.findNestedResourceFinder("POINT_CLOUD_PREDICTION");
    const std::size_t pc_pred_num_samples = rf_point_cloud_prediction.check("number_samples", Value("100")).asInt();
    const bool test_superquadrics = rf_point_cloud_prediction.check("test_superquadrics", Value(false)).asBool();

    /* Point cloud filtering. */
    ResourceFinder rf_point_cloud_filtering = rf.findNestedResourceFinder("POINT_CLOUD_FILTERING");
    const double pc_outlier_threshold = rf_point_cloud_filtering.check("outlier_threshold", Value("0.1")).asDouble();

    /* Depth. */
    ResourceFinder rf_depth = rf.findNestedResourceFinder("DEPTH");
    const std::string depth_fetch_mode = rf_depth.check("fetch_mode", Value("new_image")).toString();
    const std::size_t depth_stride     = rf_depth.check("stride", Value(1)).asInt();

    /* Hand occlusion. */
    ResourceFinder rf_hand_occlusion = rf.findNestedResourceFinder("HAND_OCCLUSION");
    const bool handle_hand_occlusion            = rf_hand_occlusion.check("handle_occlusion", Value(false)).asBool();
    const std::string hand_laterality_occlusion = rf_hand_occlusion.check("laterality", Value("right")).asString();
    const double hand_occlusion_scale           = rf_hand_occlusion.check("hull_scale", Value(1.0)).asDouble();

    /* Hand contacts. */
    ResourceFinder rf_hand_contacts = rf.findNestedResourceFinder("HAND_CONTACTS");
    const bool handle_hand_contacts                      = rf_hand_contacts.check("handle_contacts", Value(false)).asBool();
    const std::string hand_laterality_contacts           = rf_hand_contacts.check("laterality", Value()).asString();
    std::vector<std::string> used_fingers_contacts;
    if (handle_hand_contacts)
        used_fingers_contacts = loadListString(rf_hand_contacts, "used_fingers");

    /* Mesh parameters. */
    ResourceFinder rf_object = rf.findNestedResourceFinder("OBJECT");
    const std::string object_name          = rf_object.check("object_name", Value("ycb_mustard")).asString();
    const std::string object_mesh_path_obj = rf.findPath("mesh/" + object_name) + "/nontextured.obj";
    const std::string object_mesh_path_ply = rf.findPath("mesh/" + object_name) + "/nontextured.ply";

    /* Segmentation parameters. */
    Vector4d bbox_0;
    ResourceFinder rf_segmentation    = rf.findNestedResourceFinder("SEGMENTATION");
    const std::string segmentation_type     = rf_segmentation.check("type", Value("in_hand")).asString();
    const std::string iol_object_name       = rf_segmentation.check("iol_object_name", Value("Bottle")).asString();
    const double iol_bbox_scale             = rf_segmentation.check("iol_bbox_scale", Value(1.0)).asDouble();
    const bool use_initial_bounding_box     = rf_segmentation.check("use_bbox_0", Value(false)).asBool();
    if (use_initial_bounding_box)
    {
        bbox_0.head<2>() = loadVectorDouble(rf_segmentation, "bbox_tl_0", 2);
        bbox_0.tail<2>() = loadVectorDouble(rf_segmentation, "bbox_br_0", 2);
    }
    std::string mask_name;
    if (segmentation_type == "mask")
        mask_name = rf_segmentation.check("mask_name", Value("006_mustard_bottle")).asString();

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
    ResourceFinder rf_misc = rf.findNestedResourceFinder("MISC");
    const bool enable_send_hull = rf_misc.check("send_hull", Value(false)).asBool();
    const bool enable_send_mask = rf_misc.check("send_mask", Value(false)).asBool();

    /* Log parameters. */
    yInfo() << log_ID << "Robot name:" << robot;

    yInfo() << log_ID << "Particles:";
    yInfo() << log_ID << "- number:"             << number_particles;
    yInfo() << log_ID << "- resample_threshold:" << resampling_threshold;

    yInfo() << log_ID << "Likelihood:";
    yInfo() << log_ID << "- variance:" << likelihood_variance;

    yInfo() << log_ID << "Initial conditions:";
    yInfo() << log_ID << "- center_0: "      << eigenToString(center_0);
    yInfo() << log_ID << "- radius_0: "      << eigenToString(radius_0);
    yInfo() << log_ID << "- cov_x_0: "       << eigenToString(cov_x_0);
    yInfo() << log_ID << "- cov_v_0: "       << eigenToString(cov_v_0);
    yInfo() << log_ID << "- cov_eul_0: "     << eigenToString(cov_eul_0);
    yInfo() << log_ID << "- cov_eul_dot_0: " << eigenToString(cov_eul_dot_0);

    yInfo() << log_ID << "Camera:";
    yInfo() << log_ID << "- name:"         << camera_name;
    yInfo() << log_ID << "- laterality:"   << camera_laterality;
    yInfo() << log_ID << "- fallback_key:" << camera_fallback_key;

    yInfo() << log_ID << "Kinematic model:";
    yInfo() << log_ID << "- q_x:"         << eigenToString(kin_q_x);
    yInfo() << log_ID << "- q_eul:"       << eigenToString(kin_q_eul);

    yInfo() << log_ID << "Measurement model:";
    yInfo() << log_ID << "- visual_covariance:" << eigenToString(visual_covariance);
    yInfo() << log_ID << "- tactile_covariance:" << eigenToString(tactile_covariance);

    yInfo() << log_ID << "Unscented transform:";
    yInfo() << log_ID << "- alpha:" << ut_alpha;
    yInfo() << log_ID << "- beta:"  << ut_beta;
    yInfo() << log_ID << "- kappa:" << ut_kappa;

    yInfo() << log_ID << "Point cloud prediction:";
    yInfo() << log_ID << "- num_samples:" << pc_pred_num_samples;
    yInfo() << log_ID << "- test_superquadrics:" << test_superquadrics;

    yInfo() << log_ID << "Point cloud filtering:";
    yInfo() << log_ID << "- outlier_threshold:" << pc_outlier_threshold;

    yInfo() << log_ID << "Depth:";
    yInfo() << log_ID << "- fetch_mode:" << depth_fetch_mode;
    yInfo() << log_ID << "- stride:" << depth_stride;

    yInfo() << log_ID << "Hand occlusion:";
    yInfo() << log_ID << "- handle_occlusion:" << handle_hand_occlusion;
    yInfo() << log_ID << "- hull_scale:" << hand_occlusion_scale;

    yInfo() << log_ID << "Object:";
    yInfo() << log_ID << "- object_name:"        << object_name;
    yInfo() << log_ID << "- mesh path is (obj):" << object_mesh_path_obj;
    yInfo() << log_ID << "- mesh path is (ply):" << object_mesh_path_ply;

    yInfo() << log_ID << "Segmentation:";
    yInfo() << log_ID << "- type:" << segmentation_type;
    yInfo() << log_ID << "- iol_object_name:" << iol_object_name;
    yInfo() << log_ID << "- iol_bbox_scale:"  << iol_bbox_scale;
    yInfo() << log_ID << "- use_bbox_0:"      << use_initial_bounding_box;
    if (use_initial_bounding_box)
    {
        yInfo() << log_ID << "- bbox_tl_0:" << eigenToString(bbox_0.head<2>());
        yInfo() << log_ID << "- bbox_br_0:" << eigenToString(bbox_0.tail<2>());
    }

    yInfo() << log_ID << "Logging:";
    yInfo() << log_ID << "- enable_log:"        << enable_log;
    yInfo() << log_ID << "- absolute_log_path:" << log_path;

    yInfo() << log_ID << "Miscellaneous:";
    yInfo() << log_ID << "- send_hull:" << enable_send_hull;
    yInfo() << log_ID << "- send_mask:" << enable_send_mask;

    /**
     * Initialize camera
     */
    std::unique_ptr<Camera> camera;
    if (camera_name == "iCubCamera")
    {
        camera = std::unique_ptr<iCubCamera>
        (
            new iCubCamera(camera_laterality, port_prefix, "object-tracking", camera_fallback_key)
        );
    }
    else if (camera_name == "RealsenseCamera")
    {
        camera = std::unique_ptr<RealsenseCamera>
        (
            new RealsenseCamera(port_prefix, "object-tracking", camera_fallback_key)
        );
    }
    else
    {
        yError() << log_ID << "The requested camera is not available. Requested camera is" << camera_name;
        std::exit(EXIT_FAILURE);
    }
    camera->initialize();

    /**
     * Initialize object segmentation.
     */
    std::shared_ptr<PointCloudSegmentation> segmentation;
    if (segmentation_type == "in_hand")
    {
        /**
         * Initialize object rendering engine.
         */
        const std::string sicad_shader_path = rf.findPath("shader/");
        std::unique_ptr<ObjectRenderer> object_renderer = std::unique_ptr<ObjectRenderer>
        (
            new ObjectRenderer(object_mesh_path_obj, sicad_shader_path, *camera)
        );

        if (use_initial_bounding_box)
        {
            segmentation = std::make_shared<InHandObjectSegmentation>(port_prefix, depth_stride, std::move(object_renderer), bbox_0);
        }
        else
        {
            segmentation = std::make_shared<InHandObjectSegmentation>(port_prefix, depth_stride, std::move(object_renderer), iol_object_name, iol_bbox_scale);
        }

        if (handle_hand_occlusion)
        {
            /* Initialize iCubArmModel providing the 3D pose of the hand parts relative to the hand palm. */
            std::unique_ptr<iCubArmModel> icub_arm = std::unique_ptr<iCubArmModel>
            (
                new iCubArmModel(true, false, hand_laterality_occlusion, "object-tracking", "object-tracking/icub-arm-model/occlusion/" + hand_laterality_occlusion)
            );

            /* Initialize iCubHandOcclusion that reads the hand palm pose from a port
               and creates an occlusion mask to be used to clean part of the point cloud of the object
               from undesired parts due to hand occlusion. */
            std::unique_ptr<iCubHandOcclusion> hand_occlusion = std::unique_ptr<iCubHandOcclusion>
            (
                new iCubHandOcclusion(std::move(icub_arm), "object-tracking/icub-hand-occlusion/" + hand_laterality_occlusion, "left", hand_occlusion_scale)
            );

            /* Add the occlusion to the segmentation class. */
            segmentation->addObjectOcclusion(std::move(hand_occlusion));
        }
    }
    else if (segmentation_type == "bounding_box")
    {
        if (!use_initial_bounding_box)
        {
            yError() << log_ID << "With segmentation type == bounding_box, it is required to specify an initial bounding box!";
            return EXIT_FAILURE;
        }
        segmentation = std::unique_ptr<BoundingBoxSegmentation>
        (
            new BoundingBoxSegmentation(port_prefix, bbox_0, depth_stride)
        );
    }
    else if (segmentation_type == "mask")
    {
        segmentation = std::unique_ptr<MaskSegmentation>
        (
            new MaskSegmentation(port_prefix, mask_name, depth_stride)
        );
    }
    else
    {
        yError() << log_ID << "The requested segmentatio is not available. Requested segmentation type is" << segmentation_type;
        std::exit(EXIT_FAILURE);
    }

    /**
     * Initialize point cloud prediction.
     */
    std::unique_ptr<ObjectSampler> obj_sampler;
#ifdef USE_SUPERQUADRICLIB
    if (test_superquadrics)
    {
        Superquadric superq;
        VectorXd parameters = MatrixXd(11, 1);
        parameters *= 0.0;
        parameters(0) = 0.0626792;
        parameters(1) = 0.029448;
        parameters(2) = 0.101533;
        parameters(3) = 0.55835;
        parameters(4) = 0.935447;
        superq.setSuperqParams(parameters);
        obj_sampler = std::unique_ptr<SuperquadricSampler>(new SuperquadricSampler(superq));
    }
    else
#endif
    {
        if (test_superquadrics)
        {
            yError() << log_ID << "Superquadrics requested, however the feature is not compiled.";
            std::exit(EXIT_FAILURE);
        }
        obj_sampler = std::unique_ptr<ObjectMeshSampler>(new ObjectMeshSampler(object_mesh_path_ply));
    }

    std::shared_ptr<PointCloudPrediction> point_cloud_prediction = std::make_shared<NanoflannPointCloudPrediction>(std::move(obj_sampler), pc_pred_num_samples);
    if (!test_superquadrics)
    {
        if (!point_cloud_prediction->init())
        {
            yError() << log_ID << "Cannot initialize point cloud prediction object.";
            std::exit(EXIT_FAILURE);
        }
    }

    /**
     * Initialize measurement model.
     */
    std::unique_ptr<AdditiveMeasurementModel> measurement_model;
    if (robot == "icub")
    {
        if (handle_hand_contacts)
        {
            std::unique_ptr<iCubArmModel> icub_arm = std::unique_ptr<iCubArmModel>
            (
                new iCubArmModel(false, false, hand_laterality_contacts, "object-tracking", "object-tracking/icub-arm-model/contacts/" + hand_laterality_contacts)
            );

            std::unique_ptr<iCubSpringyFingersDetection> icub_springy_fingers = std::unique_ptr<iCubSpringyFingersDetection>
            (
                new iCubSpringyFingersDetection("right")
            );

            std::unique_ptr<iCubHandContactsModel> icub_contacts = std::unique_ptr<iCubHandContactsModel>
            (
                new iCubHandContactsModel(std::move(icub_arm), std::move(icub_springy_fingers), used_fingers_contacts, "object-tracking/icub-hand-contacts")
            );

            measurement_model = std::unique_ptr<iCubObjectMeasurements>
            (
                new iCubObjectMeasurements(std::move(camera), segmentation, std::move(point_cloud_prediction), std::move(icub_contacts), visual_covariance_diagonal, tactile_covariance_diagonal, pc_outlier_threshold, depth_fetch_mode)
            );
        }
        else
        {
            measurement_model = std::unique_ptr<iCubObjectMeasurements>
            (
                new iCubObjectMeasurements(std::move(camera), segmentation, std::move(point_cloud_prediction), visual_covariance_diagonal, pc_outlier_threshold, depth_fetch_mode)
            );
        }

    }
    else if (robot == "r1")
    {
        measurement_model = std::unique_ptr<ObjectMeasurements>
        (
            new ObjectMeasurements(std::move(camera), segmentation, std::move(point_cloud_prediction), visual_covariance_diagonal, pc_outlier_threshold, depth_fetch_mode)
        );
    }

    if (enable_log)
        measurement_model->enable_log(log_path, "object-tracking");

    /**
     * Filter construction.
     */

    /**
     * StateModel
     */
    std::unique_ptr<LinearStateModel> kinematic_model;
    if (use_tdd_occlusion)
    {
        kinematic_model = std::unique_ptr<DiscretizedKinematicModelTDD>
        (
            new DiscretizedKinematicModelTDD(kin_q_x(0), kin_q_x(1), kin_q_x(2), kin_q_eul(0), kin_q_eul(1), kin_q_eul(2), tdd_exp_gain, tdd_max_seconds)
        );
    }
    else
    {
        kinematic_model = std::unique_ptr<DiscretizedKinematicModel>
        (
            new DiscretizedKinematicModel(kin_q_x(0), kin_q_x(1), kin_q_x(2), kin_q_eul(0), kin_q_eul(1), kin_q_eul(2))
        );
    }

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

    /**
     * Prediction step.
     */
    std::unique_ptr<GaussianPrediction> prediction = std::unique_ptr<KFPrediction>
    (
        new KFPrediction(std::move(kinematic_model))
    );

    /**
     * Correction step.
     */
    std::size_t measurement_sub_size = 3; // i.e. a measurement is made of 3 * N points
                                          // where N is the number of points originated from the object

    std::unique_ptr<Correction> correction = std::unique_ptr<Correction>
    (
        new Correction(std::move(measurement_model), state_size, ut_alpha, ut_beta, ut_kappa, measurement_sub_size)
    );

    /**
     * Filter.
     */
    std::cout << "Initializing filter..." << std::flush;

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
    std::unique_ptr<ObjectSampler> obj_sampler_likelihood = std::unique_ptr<ObjectMeshSampler>(
        new ObjectMeshSampler(object_mesh_path_ply));
    std::unique_ptr<NanoflannPointCloudPrediction> distances_approximation = std::unique_ptr<NanoflannPointCloudPrediction>(
        new NanoflannPointCloudPrediction(std::move(obj_sampler_likelihood), pc_pred_num_samples));

    std::unique_ptr<ProximityLikelihood> proximity_likelihood = std::unique_ptr<ProximityLikelihood>(
        new ProximityLikelihood(likelihood_variance, std::move(distances_approximation)));

    pf_correction = std::unique_ptr<ParticlesCorrection>(
        new ParticlesCorrection(std::move(correction), std::move(proximity_likelihood)));

    /* Resampling. */
    std::unique_ptr<Resampling> pf_resampling = std::unique_ptr<Resampling>(new Resampling());

    std::unique_ptr<FilteringAlgorithm> filter = std::unique_ptr<PFilter>
    (
        new PFilter(port_prefix,
                    number_particles, resampling_threshold,
                    point_estimate_method, point_estimate_window_size,
                    std::move(pf_initialization),
                    std::move(pf_prediction), std::move(pf_correction),
                    std::move(pf_resampling),
                    std::move(segmentation))
    );
    std::cout << "done." << std::endl;

    if (enable_log)
        filter->enable_log(log_path, "object-tracking");

    std::cout << "Booting filter..." << std::flush;

    filter->boot();

    std::cout << "done." << std::endl;

    std::cout << "Running filter..." << std::endl;

    if (!filter->wait())
        return EXIT_FAILURE;

    yInfo() << log_ID << "Application closed succesfully.";

    return EXIT_SUCCESS;
}

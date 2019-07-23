/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <iCubCamera.h>
#include <LocalizeSuperquadricSampler.h>
#include <RealsenseCamera.h>
#include <VCGTriMesh.h>
#include <YCBVideoCamera.h>
#include <YCBVideoCameraNRT.h>

#include <vtkFitImplicitFunction.h>
#include <vtkBoundedPointSource.h>

#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>

#include <iostream>

#include <thrift/LocalizeSuperquadricSamplerIDL.h>

using namespace Eigen;
using namespace bfl;
using namespace yarp::os;
using namespace yarp::sig;


LocalizeSuperquadricSampler::LocalizeSuperquadricSampler
(
    const std::string& port_prefix,
    const std::string& camera_name,
    const std::string& camera_fallback_key,
    const std::string& camera_laterality
)
{
    // Open port for communication with module localize-superquadrics
    if (!(localize_superq_rpc_client_.open("/" + port_prefix + "/localize_superquadric_rpc:o")))
    {
        std::string err = log_ID_ + "::ctor. Error: cannot open superquadric rpc client port.";
        throw(std::runtime_error(err));
    }

    // Open port for communication with module object-tracking-viewer
    if (!(viewer_rpc_client_.open("/" + port_prefix + "/viewer-rpc:o")))
    {
        std::string err = log_ID_ + "::ctor. Error: cannot open mask rpc client port.";
        throw(std::runtime_error(err));
    }

    // Open RPC input port for commands
    if (!port_rpc_command_.open("/" + port_prefix + "/cmd:i"))
    {
        std::string err = log_ID_ + "::ctor. Error: cannot open RPC input command port.";
        throw(std::runtime_error(err));
    }

    if (!(this->yarp().attachAsServer(port_rpc_command_)))
    {
        std::string err = "PFILTER::CTOR::ERROR\n\tError: cannot attach the RPC command port.";
        throw(std::runtime_error(err));
    }

    // Initialize camera required to extract depth
    if (camera_name == "iCubCamera")
    {
        camera_ = std::unique_ptr<iCubCamera>(new iCubCamera(camera_laterality, port_prefix, "object-tracking", camera_fallback_key));
    }
    else if (camera_name == "RealsenseCamera")
    {
        camera_ = std::unique_ptr<RealsenseCamera>(new RealsenseCamera(port_prefix, "object-tracking", camera_fallback_key));
    }
    else if (camera_name == "YCBVideoCamera")
    {
        camera_ = std::unique_ptr<YcbVideoCamera>(new YcbVideoCamera(port_prefix, "object-tracking", camera_fallback_key));
    }
    else
    {
        std::string err = log_ID_ + "::ctor. The requested camera is not available. Requested camera is " + camera_name;
        throw(std::runtime_error(err));
    }
    camera_->initialize();

    // Initialize mask segmentation required to obtain the point cloud of the instance of interest
    // Second argument is mask name - unknown at the moment
    // Third argument is depth stride - set to 1 - becuase the superquadric modelling pipeline
    // will take care of subsampling the point cloud
    // segmentation_ = std::unique_ptr<MaskSegmentation>(new MaskSegmentation(port_prefix, "", 2, true));
    segmentation_ = std::unique_ptr<MaskSegmentation>(new MaskSegmentation(port_prefix, "", 4, false));
}


LocalizeSuperquadricSampler::LocalizeSuperquadricSampler(const std::string& port_prefix, const std::string& mask_path, const std::string& camera_path)
{
    // Open port for communication with module localize-superquadrics
    if (!(localize_superq_rpc_client_.open("/" + port_prefix + "/localize_superquadric_rpc:o")))
    {
        std::string err = log_ID_ + "::ctor. Error: cannot open superquadric rpc client port.";
        throw(std::runtime_error(err));
    }

    // Open port for communication with module object-tracking-viewer
    if (!(viewer_rpc_client_.open("/" + port_prefix + "/viewer-rpc:o")))
    {
        std::string err = log_ID_ + "::ctor. Error: cannot open mask rpc client port.";
        throw(std::runtime_error(err));
    }

    // Open RPC input port for commands
    if (!port_rpc_command_.open("/" + port_prefix + "/cmd:i"))
    {
        std::string err = log_ID_ + "::ctor. Error: cannot open RPC input command port.";
        throw(std::runtime_error(err));
    }

    if (!(this->yarp().attachAsServer(port_rpc_command_)))
    {
        std::string err = "PFILTER::CTOR::ERROR\n\tError: cannot attach the RPC command port.";
        throw(std::runtime_error(err));
    }

    camera_ = std::unique_ptr<YcbVideoCameraNrt>(new YcbVideoCameraNrt(camera_path, 320, 240, "object-tracking"));
    camera_->initialize();

    // Initialize mask segmentation required to obtain the point cloud of the instance of interest
    // Second argument is mask name - unknown at the moment
    // Third argument is depth stride - set to 1 - becuase the superquadric modelling pipeline
    // will take care of subsampling the point cloud
    segmentation_ = std::unique_ptr<MaskSegmentation>(new MaskSegmentation(port_prefix, mask_path, "", 2));
}


LocalizeSuperquadricSampler::~LocalizeSuperquadricSampler()
{
    localize_superq_rpc_client_.close();
    viewer_rpc_client_.close();
    port_rpc_command_.close();
}


std::pair<bool, MatrixXd> LocalizeSuperquadricSampler::sample(const std::size_t& number_of_points)
{
    // Try to freeze segmentation, until it is ready
    bool valid_segmentation = false;
    while (!valid_segmentation)
    {
        // Freeze camera
        camera_->freeze();

        std::cout << log_ID_ << "::sample. Trying to get segmentation from instance semgmentation module." << std::endl;
        valid_segmentation = segmentation_->freezeSegmentation(*camera_);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << log_ID_ << "::sample. Valid segmentation obtained for object " << object_name_ << std::endl;

    // Then get depth. Of course the time stamp of the depth will be different from that of the segmentation.
    // However, we are assuming a static scenario during initialization.
    bool valid_depth;
    MatrixXf depth;
    std::tie(valid_depth, depth) = camera_->getDepthImage(true);
    if (!valid_depth)
    {
        std::string err = log_ID_ + "::sample. Error: unable to get depth from camera.";
        throw(std::runtime_error(err));
    }
    std::cout << log_ID_ << "::sample. Depth obtained succesfully." << std::endl;

    // Once done, extract the point cloud
    bool valid_point_cloud;
    MatrixXd point_cloud;
    std::tie(valid_point_cloud, point_cloud) = segmentation_->extractPointCloud(*camera_, depth, 1.0);
    if (!valid_point_cloud)
    {
        std::string err = log_ID_ + "::sample. Error: unable to extract point cloud from segmented image.";
        throw(std::runtime_error(err));
    }
    std::cout << log_ID_ << "::sample. Point cloud extracted succesfully." << std::endl;

    // Evaluate centroid of point cloud and discard too far points
    VectorXd centroid = (point_cloud.rowwise().sum()) / point_cloud.cols();
    VectorXi good_points(point_cloud.cols());
    double threshold;
    // if (object_name_ == "Cracker" || (object_name_.find("Bottle") != std::string::npos))
    //     threshold = 0.2;
    // else
    //     threshold = 0.1;
    for (int i = 0 ; i < point_cloud.cols(); i++)
    {
        // good_points(i) = 0;
        // if ((point_cloud.col(i) - centroid).norm() < threshold)
            good_points(i) = 1;
    }

    // Need to convert the point cloud to a YARP point cloud
    PointCloud<DataXYZRGBA> yarp_point_cloud;
    yarp_point_cloud.resize(good_points.sum());
    for (std::size_t i = 0, j = 0; i < point_cloud.cols(); i++)
    {
        if (good_points(i) == 1)
        {
            // out << point_cloud.col(i).transpose() << std::endl;
            yarp_point_cloud(j).x = point_cloud(0, i);
            yarp_point_cloud(j).y = point_cloud(1, i);
            yarp_point_cloud(j).z = point_cloud(2, i);
            yarp_point_cloud(j).r = 255;
            yarp_point_cloud(j).g = 0;
            yarp_point_cloud(j).b = 0;

            j++;
        }
    }
    // out.close();

    // Get the superquadric from module localize-supequadrics
    bool valid_superquadric = false;
    vtkSmartPointer<vtkSuperquadric> superquadric;
    std::tie(valid_superquadric, superquadric) = getSuperquadricFromRpc(yarp_point_cloud);
    if (!valid_superquadric)
    {
        std::string err = log_ID_ + "::sample. Error: unable to get superquadric from module localize-superquadric.";
        throw(std::runtime_error(err));
    }
    std::cout << log_ID_ << "::sample. Superquadric obtained successfully." << std::endl;

    // Find points belonging to the surface of the superquadric using VTK

    // Initialize samples around the superquadric
    vtkMath::RandomSeed(1);
    vtkSmartPointer<vtkBoundedPointSource> samples = vtkSmartPointer<vtkBoundedPointSource>::New();
    samples->SetNumberOfPoints(10000000);
    double superq_scales[3];
    superquadric->GetScale(superq_scales);
    double scale_multiplier = 1.0 / superquadric->GetSize();
    superquadric->SetSize(1.0);
    double bound_x = (superq_scales[0] * scale_multiplier * 1.1) / 2.0;
    double bound_y = (superq_scales[1] * scale_multiplier * 1.1) / 2.0;
    double bound_z = (superq_scales[2] * scale_multiplier * 1.1) / 2.0;
    samples->SetBounds(-bound_x, bound_x, -bound_y, bound_y, -bound_z, bound_z);

    // Find samples almost on the surface of the superquadric
    vtkSmartPointer<vtkFitImplicitFunction> fit = vtkSmartPointer<vtkFitImplicitFunction>::New();
    fit->SetInputConnection(samples->GetOutputPort());
    fit->SetImplicitFunction(superquadric);
    fit->SetThreshold(0.01);
    fit->Update();

    // Rotate points since the VTK superquadrics uses swapped y/z axes
    auto vtk_points = fit->GetOutput();
    MatrixXd sampled_points(3, vtk_points->GetNumberOfPoints());
    for (std::size_t i = 0; i < sampled_points.cols(); i++)
    {
        double* ptr = vtk_points->GetPoint(i);
        sampled_points.col(i) << ptr[0], ptr[1], ptr[2];
    }
    Matrix3d rot_x(AngleAxisd(- M_PI / 2.0, Vector3d::UnitX()));
    sampled_points = rot_x * sampled_points;

    std::cout << log_ID_ << "::sample. Superquadric sampled succesfully." << std::endl;

    // Perform Disk Poisson Sampling

    // Copy points into mesh vertices
    simpleTriMesh vcg_sampled_points;
    VertexIterator vi = simpleTriMeshAllocator::AddVertices(vcg_sampled_points, sampled_points.cols());
    for (std::size_t i = 0; i < sampled_points.cols(); i++)
    {
        (*vi).P()[0] = sampled_points(0, i);
        (*vi).P()[1] = sampled_points(1, i);
        (*vi).P()[2] = sampled_points(2, i);
        vi++;
    }

    // Perform bounding box computation
    vcg::tri::UpdateBounding<simpleTriMesh>::Box(vcg_sampled_points);

    // Perform normal computation
    // Some default parametrs as found in MeshLab
    simpleTriMeshPointCloudNormal::Param normal_estimation_params;
    normal_estimation_params.fittingAdjNum = 10;
    normal_estimation_params.smoothingIterNum = 0;

    // This will force normals to be pointing to the inner of the superquadric
    vcgVector viewPoint;
    viewPoint.SetZero();
    normal_estimation_params.viewPoint = viewPoint;
    normal_estimation_params.useViewPoint = true;

    simpleTriMeshPointCloudNormal::Compute(vcg_sampled_points, normal_estimation_params, nullptr);

    // Some default parametrs as found in MeshLab
    // std::size_t oversampling = 20;
    triMeshSurfSampler::PoissonDiskParam poiss_params;
    poiss_params.radiusVariance = 1;
    poiss_params.geodesicDistanceFlag = false;
    poiss_params.bestSampleChoiceFlag = true;
    poiss_params.bestSamplePoolSize = 10;

    // Estimate radius required to obtain disk poisson sampling with the number_of_points points
    simpleTriMesh::ScalarType radius;
    radius = triMeshSurfSampler::ComputePoissonDiskRadius(vcg_sampled_points, number_of_points);

    // Generate disk poisson samples by pruning the existing samples
    simpleTriMesh poiss_mesh;
    triMeshSampler dp_sampler(poiss_mesh);
    triMeshSurfSampler::PoissonDiskPruningByNumber(dp_sampler, vcg_sampled_points, number_of_points, radius, poiss_params, 0.005);
    vcg::tri::UpdateBounding<simpleTriMesh>::Box(poiss_mesh);

    std::cout << log_ID_ << "::sample. Samples decimated using PoissonDisk sampling." << std::endl;

    // Store the cloud
    std::size_t number_points = std::distance(poiss_mesh.vert.begin(), poiss_mesh.vert.end());
    MatrixXd poisson_cloud(6, number_points);
    std::size_t i = 0;
    for (VertexIterator vi = poiss_mesh.vert.begin(); vi != poiss_mesh.vert.end(); vi++)
    {
        // Extract the point
        const auto p = vi->cP();

        // Extract the normal
        // Normals need to be swapped because the view point was set in (0, 0, 0)
        const auto n = vi->cN();
        Vector3d normal(-n[0], -n[1], -n[2]);
        normal.normalize();

        // Convert the normal to latitude, longitude
        double phi = std::atan2(normal(2), std::sqrt(normal(0) * normal(0) + normal(1) * normal(1)));
        double lambda = std::atan2(normal(1), normal(0));

        // Add to the cloud
        poisson_cloud.col(i) << p[0], p[1], p[2], phi, lambda;

        i++;
    }

    return std::make_pair(true, poisson_cloud);
}


void LocalizeSuperquadricSampler::setObjectName(const std::string& object_name)
{
    object_name_ = object_name;
    segmentation_->setMaskName(getObjectMaskName(object_name));
}


Eigen::VectorXd LocalizeSuperquadricSampler::getObjectPose()
{
    return object_pose_;
}


std::unique_ptr<ParticleSetInitialization> LocalizeSuperquadricSampler::getParticleSetInitialization()
{
    return std::unique_ptr<LocalizeSuperquadricInitializer>(new LocalizeSuperquadricInitializer(*this));
}


bool LocalizeSuperquadricSampler::send_superquadric_to_viewer()
{
    return sendSuperquadricToViewer();
}


std::pair<bool, vtkSmartPointer<vtkSuperquadric>> LocalizeSuperquadricSampler::getSuperquadricFromRpc(const PointCloud<DataXYZRGBA>& yarp_point_cloud)
{
    vtkSmartPointer<vtkSuperquadric> superquadric = vtkSmartPointer<vtkSuperquadric>::New();
    Bottle cmd, reply;
    cmd.clear();

    // Compose command for localize-superquadrics
    cmd.addString("localize_superq");
    cmd.addString(object_name_);
    Bottle& point_cloud_list = cmd.addList();
    point_cloud_list = yarp_point_cloud.toBottle();
    cmd.addInt(0);

    reply.clear();
    localize_superq_rpc_client_.write(cmd, reply);

    if (reply.size() == 0)
    {
        std::cout << log_ID_ << "::getSuperquadricFromRpc. Empty response from module localize-superquarics" << std::endl;
        return std::make_pair(false,  superquadric);
    }

    // Parameters format: center-x center-y center-z angle-z angle-y angle-z size-x size-y size-z epsilon-1 epsilon-2
    Bottle parameters = *(reply.get(0).asList());

    superquadric->ToroidalOff();

    // This is not a typo, 6-8-7 is the correct sequence
    superquadric->SetPhiRoundness(parameters.get(9).asDouble());
    superquadric->SetThetaRoundness(parameters.get(10).asDouble());
    superquadric->SetScale(parameters.get(6).asDouble(),parameters.get(8).asDouble(),parameters.get(7).asDouble());

    // Store the object pose
    object_pose_.resize(6);
    object_pose_(0) = parameters.get(0).asDouble();
    object_pose_(1) = parameters.get(1).asDouble();
    object_pose_(2) = parameters.get(2).asDouble();
    object_pose_(3) = parameters.get(5).asDouble() * M_PI / 180.0;
    object_pose_(4) = parameters.get(4).asDouble() * M_PI / 180.0;
    object_pose_(5) = parameters.get(3).asDouble() * M_PI / 180.0;

    // Store parameters
    superquadric_parameters_.resize(11);
    superquadric_parameters_(0) = parameters.get(6).asDouble();
    superquadric_parameters_(1) = parameters.get(8).asDouble();
    superquadric_parameters_(2) = parameters.get(7).asDouble();
    superquadric_parameters_(3) = parameters.get(9).asDouble();
    superquadric_parameters_(4) = parameters.get(10).asDouble();
    superquadric_parameters_.tail<6>() = object_pose_;

    sendSuperquadricToViewer();

    return std::make_pair(true, superquadric);
}

std::string LocalizeSuperquadricSampler::getObjectMaskName(const std::string& object_name)
{
    std::string name = "";

    if (object_name == "Cracker")
        name = "003_cracker_box";
    else if (object_name == "Domino")
        name = "004_sugar_box";
    else if (object_name == "SpamCan")
        name = "010_potted_meat_can";
    else if (object_name == "CampbellCan")
        name = "005_tomato_soup_can";
    else if (object_name == "Mustard")
        name = "006_mustard_bottle";
    else if (object_name == "BottleMustard")
        name = "bottle_mustard";
    else if (object_name == "BottleOrange")
        name = "bottle_orange";
    else if (object_name == "BottleIit")
        name = "bottle_iit";
    else if (object_name == "BottlePinkTea")
        name = "bottle_pinktea";
    else if (object_name == "BottleGreenTea")
        name = "bottle_greentea";

    return name;
}


bool LocalizeSuperquadricSampler::sendSuperquadricToViewer()
{
    Bottle cmd, reply;

    // Send received parameters also to module object-tracking-viewer
    cmd.clear();
    cmd.addString("use_superquadric");
    cmd.addDouble(superquadric_parameters_(0));
    cmd.addDouble(superquadric_parameters_(1));
    cmd.addDouble(superquadric_parameters_(2));
    cmd.addDouble(superquadric_parameters_(3));
    cmd.addDouble(superquadric_parameters_(4));
    cmd.addDouble(superquadric_parameters_(5));
    cmd.addDouble(superquadric_parameters_(6));
    cmd.addDouble(superquadric_parameters_(7));
    cmd.addDouble(superquadric_parameters_(8));
    cmd.addDouble(superquadric_parameters_(9));
    cmd.addDouble(superquadric_parameters_(10));

    reply.clear();
    return viewer_rpc_client_.write(cmd, reply);
}


LocalizeSuperquadricInitializer::LocalizeSuperquadricInitializer(LocalizeSuperquadricSampler& sampler) :
    sampler_(sampler)
{ }


LocalizeSuperquadricInitializer::~LocalizeSuperquadricInitializer()
{ }


bool LocalizeSuperquadricInitializer::initialize(bfl::ParticleSet& particles)
{
    VectorXd initial_pose = sampler_.getObjectPose();
    VectorXd state = VectorXd::Zero(12);
    state.head<3>() = initial_pose.head<3>();
    state.tail<3>() = initial_pose.tail<3>();

    MatrixXd initial_covariance = MatrixXd::Identity(12, 12) * 0.01;

    for (int i = 0; i < particles.state().cols(); i++)
    {
        // Initialize mean state with zero velocities
        particles.mean(i) = state;

        // Initialize state covariance
        particles.covariance(i) = initial_covariance;
    }

    // Set particles position the same as the mean state
    // TODO: they should be sampled from the initial gaussian
    particles.state() = particles.mean();

    // Initialize weights
    particles.weight().fill(-std::log(particles.state().cols()));

    return true;
}

#include <SimulatedPointCloud.h>

using namespace bfl;
using namespace Eigen;

SimulatedPointCloud::SimulatedPointCloud
(
    const std::string& mesh_filename,
    std::unique_ptr<PointCloudPrediction> prediction,
    std::unique_ptr<SimulatedStateModel> simulated_state_model,
    const Eigen::Ref<const Eigen::MatrixXd>& model_noise_covariance,
    const Eigen::Ref<const Eigen::VectorXd>& observer_origin,
    const std::size_t number_of_points,
    const bool enable_back_culling
):
    SimulatedPointCloud
    (
        mesh_filename,
        std::move(prediction),
        std::move(simulated_state_model),
        model_noise_covariance,
        observer_origin,
        number_of_points,
        enable_back_culling,
        1
    )
{ }

SimulatedPointCloud::SimulatedPointCloud
(
    const std::string& mesh_filename,
    std::unique_ptr<PointCloudPrediction> prediction,
    std::unique_ptr<SimulatedStateModel> simulated_state_model,
    const Eigen::Ref<const Eigen::MatrixXd>& model_noise_covariance,
    const Eigen::Ref<const Eigen::VectorXd>& observer_origin,
    const std::size_t number_of_points,
    const bool enable_back_culling,
    unsigned int seed
):
    MeshImporter(mesh_filename),
    PointCloudModel(std::move(prediction), model_noise_covariance),
    simulated_model_(std::move(simulated_state_model)),
    observer_origin_(observer_origin),
    number_of_points_(number_of_points),
    noisy_(false),
    enable_back_culling_(enable_back_culling),
    step_(0),
    generator_(std::mt19937_64(seed)),
    distribution_(std::normal_distribution<float>(0.0, 1.0)),
    gauss_rnd_sample_([&] { return (distribution_)(generator_); })
{
    // Convert mesh using MeshImporter
    std::istringstream mesh_input;
    bool valid_mesh;

    std::tie(valid_mesh, mesh_input) = getMesh("obj");

    if (!valid_mesh)
    {
        std::string err = "NANOFLANNPOINTCLOUDPREDICTION::CTOR::ERROR\n\tError: cannot load mesh file " + mesh_filename + ".";
        throw(std::runtime_error(err));
    }

    // Open converted obj using vcg mesh importer
    OBJImportInfo info;
    int outcome;
    outcome = simpleTriMeshImporter::OpenStream(trimesh_, mesh_input, info);

    if(simpleTriMeshImporter::ErrorCritical(outcome))
    {
        std::string err = "NANOFLANNPOINTCLOUDPREDICTION::CTOR::ERROR\n\tError: cannot load mesh file " + mesh_filename +
                          ". Error:" + std::string(simpleTriMeshImporter::ErrorMsg(outcome)) + ".";
        throw(std::runtime_error(err));
    }

    // Update bounding box
    vcg::tri::UpdateBounding<simpleTriMesh>::Box(trimesh_);

    // Update face normals
    if(trimesh_.fn > 0)
        vcg::tri::UpdateNormal<simpleTriMesh>::PerFace(trimesh_);

    // Update vertex normals
    if(trimesh_.vn > 0)
	vcg::tri::UpdateNormal<simpleTriMesh>::PerVertex(trimesh_);
}


void SimulatedPointCloud::enableNoise(const double std_noise_x, const double std_noise_y, const double std_noise_z)
{
    noisy_ = true;

    // Initialize square root covariance matrix
    sqrt_noise_covariance_ = Matrix3d::Zero();

    sqrt_noise_covariance_(0, 0) = std_noise_x;
    sqrt_noise_covariance_(1, 1) = std_noise_y;
    sqrt_noise_covariance_(2, 2) = std_noise_z;
}


void SimulatedPointCloud::transformModel(const VectorXd& state, simpleTriMesh& transformed)
{
    // Duplicate the original mesh
    copyTriMesh::Mesh(transformed, trimesh_);

    // Update the bounding box
    vcg::tri::UpdateBounding<simpleTriMesh>::Box(transformed);

    // Set the translational part of the transformation
    vcgHomMatrix Htransl;
    Htransl.SetTranslate(state(0), state(1), state(2));

    // Set the rotational part of the transformation
    vcgHomMatrix Hrot;
    vcgQuaternion q(state(6), state(7), state(8), state(9));
    q.ToMatrix(Hrot);

    // Compose the homogeneous matrix
    vcgHomMatrix H = Htransl * Hrot;

    // Apply transformation to the mesh.
    transformTriMesh::Matrix(transformed, H, true);
}


bool SimulatedPointCloud::freezeMeasurements()
{
    try
    {
        // Try to get a prefetched measurement
        measurement_ = fetched_measurements_.at(step_);
    }
    catch(std::out_of_range)
    {
        // If not possible sample a new point cloud

        if (!simulated_model_->bufferData())
            return false;

        const Ref<const VectorXd>& state = any::any_cast<MatrixXd>(simulated_model_->getData());

        measurement_ = samplePointCloud(state);
    }

    logger(measurement_.transpose());

    step_++;

    return true;
}

VectorXd SimulatedPointCloud::samplePointCloud(const VectorXd& state)
{
    // Transform the model using the current pose set
    simpleTriMesh mesh_cp;
    transformModel(state, mesh_cp);

    if (enable_back_culling_)
    {
        // Perform back-face culling
        Vector3d normal(3);
        Vector3d vertex0(3);
        for (FaceIterator fi = mesh_cp.face.begin(); fi != mesh_cp.face.end(); fi++)
        {
            // Extract face normal
            const auto n = fi->cN();
            normal << n[0], n[1], n[2];

            // Extract position of first vertex of the face
            const auto v = fi->cP(0);
            vertex0 << v[0], v[1], v[2];

            // Eval vector from view point to first vertex
            VectorXd diff = vertex0 - observer_origin_;

            // Perform culling check
            if (diff.dot(normal) >= 0)
                // Remove face
                simpleTriMeshAllocator::DeleteFace(mesh_cp, *fi);
        }
        // Perform face garbage collection after culling
        simpleTriMeshAllocator::CompactFaceVector(mesh_cp);
    }

    // Perform Disk Poisson Sampling

    // Some default parametrs as found in MeshLab
    std::size_t oversampling = 20;
    triMeshSurfSampler::PoissonDiskParam poiss_params;
    poiss_params.radiusVariance = 1;
    poiss_params.geodesicDistanceFlag = false;
    poiss_params.bestSampleChoiceFlag = true;
    poiss_params.bestSamplePoolSize = 10;

    // Estimate radius required to obtain disk poisson sampling
    // with the number_of_points points
    simpleTriMesh::ScalarType radius;
    radius = triMeshSurfSampler::ComputePoissonDiskRadius(mesh_cp, number_of_points_);

    // Generate preliminar montecarlo sampling with uniform probability
    simpleTriMesh montecarlo_mesh;
    triMeshSampler mc_sampler(montecarlo_mesh);
    mc_sampler.qualitySampling=true;
    triMeshSurfSampler::Montecarlo(mesh_cp,
				   mc_sampler,
				   number_of_points_ * oversampling);
    // Copy the bounding box from the original mesh
    montecarlo_mesh.bbox = mesh_cp.bbox;

    // Generate disk poisson samples by pruning the montecarlo cloud
    simpleTriMesh poiss_mesh;
    triMeshSampler dp_sampler(poiss_mesh);
    triMeshSurfSampler::PoissonDiskPruning(dp_sampler,
					   montecarlo_mesh,
					   radius,
					   poiss_params);
    vcg::tri::UpdateBounding<simpleTriMesh>::Box(poiss_mesh);


    // Store the cloud
    std::size_t number_points = std::distance(poiss_mesh.vert.begin(), poiss_mesh.vert.end());
    VectorXd cloud(3 * number_points);
    std::size_t i = 0;
    for (VertexIterator vi = poiss_mesh.vert.begin(); vi != poiss_mesh.vert.end(); vi++)
    {
	// Extract the point
	const auto p = vi->cP();

        // Add to the cloud
        cloud.segment(i * 3, 3) << p[0], p[1], p[2];

        i++;
    }

    // Add noise if required
    if (noisy_)
        cloud += getNoiseSample(number_points);

    return cloud;
}


std::pair<bool, Data> SimulatedPointCloud::measure() const
{
    return std::make_pair(true, measurement_);
}


std::pair<std::size_t, std::size_t> SimulatedPointCloud::getOutputSize() const
{
    return std::make_pair(measurement_.size(), 0);
}


bool SimulatedPointCloud::prefetchMeasurements(const std::size_t number_of_steps)
{
    for (std::size_t i = 0; i < number_of_steps; i++)
    {
        if (!simulated_model_->bufferData())
            return false;

        const Ref<const VectorXd>& state = any::any_cast<MatrixXd>(simulated_model_->getData());

        fetched_measurements_[i] = samplePointCloud(state);
    }

    return true;
}


VectorXd SimulatedPointCloud::getNoiseSample(const std::size_t number)
{
    MatrixXd rand_vectors(3, number);
    for (int i = 0; i < rand_vectors.size(); i++)
        *(rand_vectors.data() + i) = gauss_rnd_sample_();

    MatrixXd samples = sqrt_noise_covariance_ * rand_vectors;

    return Map<VectorXd>(samples.data(), samples.rows() * samples.cols());
}

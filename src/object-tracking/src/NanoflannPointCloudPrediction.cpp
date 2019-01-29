#include <NanoflannPointCloudPrediction.h>
#ifdef _OPENMP
#include <omp.h>
#endif

using namespace Eigen;
using namespace nanoflann;


NanoflannPointCloudPrediction::NanoflannPointCloudPrediction(const std::string& mesh_filename, const std::size_t number_of_points) :
    MeshImporter(mesh_filename),
    number_of_points_(number_of_points)
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

    // Sample the point cloud once
    samplePointCloud();

    // Initialize tree
    adapted_cloud_ = std::unique_ptr<PointCloudAdaptor>(new PointCloudAdaptor(cloud_));
    tree_ = std::unique_ptr<kdTree>(new kdTree(3 /* dim */, *adapted_cloud_, KDTreeSingleIndexAdaptorParams(10 /* max leaf */)));
    tree_->buildIndex();
}


void NanoflannPointCloudPrediction::samplePointCloud()
{
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
    radius = triMeshSurfSampler::ComputePoissonDiskRadius(trimesh_, number_of_points_);

    // Generate preliminar montecarlo sampling with uniform probability
    simpleTriMesh montecarlo_mesh;
    triMeshSampler mc_sampler(montecarlo_mesh);
    mc_sampler.qualitySampling=true;
    triMeshSurfSampler::Montecarlo(trimesh_,
				   mc_sampler,
				   number_of_points_ * oversampling);
    // Copy the bounding box from the original mesh
    montecarlo_mesh.bbox = trimesh_.bbox;

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
    cloud_.resize(3, number_points);
    std::size_t i = 0;
    for (VertexIterator vi = poiss_mesh.vert.begin(); vi != poiss_mesh.vert.end(); vi++)
    {
	// Extract the point
	const auto p = vi->cP();

        // Add to the cloud
        cloud_.col(i) << p[0], p[1], p[2];

        i++;
    }
}


std::pair<bool, MatrixXd> NanoflannPointCloudPrediction::predictPointCloud(ConstMatrixXdRef state, ConstVectorXdRef meas)
{
    // Check if meas size is multiple of 3
    if ((meas.size() % 3) != 0)
        return std::make_pair(false, MatrixXd(0, 0));

    std::size_t components = meas.size() / 3;
    MatrixXd predictions(meas.size(), state.cols());

    // Reshape measurement as a matrix
    Map<const MatrixXd> meas_matrix(meas.data(), 3, components);

    // Process all the states
    MatrixXd meas_body(3, components * state.cols());
    std::vector<Transform<double, 3, Eigen::Affine>> poses(state.cols());
    #pragma omp parallel for
    for (std::size_t i = 0; i < state.cols(); i++)
    {
        // const Ref<const VectorXd> state_i = state.col(i);

        // Transform<double, 3, Eigen::Affine> pose;

        // Compose translation
        poses[i] = Translation<double, 3>(state.col(i).segment(0, 3));

        // Compose rotation
        poses[i].rotate(AngleAxis<double>(state(9,  i), Vector3d::UnitZ()));
        poses[i].rotate(AngleAxis<double>(state(10, i), Vector3d::UnitY()));
        poses[i].rotate(AngleAxis<double>(state(11, i), Vector3d::UnitX()));

        // Express measurement in body fixed frame
        meas_body.middleCols(components * i, components) = poses[i].inverse() * meas_matrix.colwise().homogeneous();
    }

    // Predict measurement
    MatrixXd pred_meas_body(3, state.cols() * components);
    #pragma omp parallel for collapse(2)
    for (std::size_t i = 0; i < state.cols(); i++)
    {
        for (std::size_t j = 0; j < components; j++)
        {
            const Ref<const Vector3d> meas_j = meas_body.middleCols(components * i, components).col(j);

            std::size_t ret_index;
            double out_dist_sqr;
            KNNResultSet<double> resultSet(1);
            resultSet.init(&ret_index, &out_dist_sqr);
            // Querying tree_ is thread safe as per this issue
            // https://github.com/jlblancoc/nanoflann/issues/54
            tree_->findNeighbors(resultSet, meas_j.data(), nanoflann::SearchParams(10));

            pred_meas_body.middleCols(components * i, components).col(j) = cloud_.col(ret_index);
        }
    }

    #pragma omp parallel for
    for (std::size_t i = 0; i < state.cols(); i++)
    {
        // Convert back in robot frame
        MatrixXd meas_robot = poses[i] * pred_meas_body.middleCols(components * i, components).colwise().homogeneous();

        // Store predicted measurement
        predictions.col(i) = std::move(Map<VectorXd>(meas_robot.data(), meas.size()));
    }

    return std::make_pair(true, predictions);
}


std::pair<bool, MatrixXd> NanoflannPointCloudPrediction::evalDistances(ConstMatrixXdRef state, ConstVectorXdRef meas)
{
    // Check if meas size is multiple of 3
    if ((meas.size() % 3) != 0)
        return std::make_pair(false, MatrixXd(0, 0));

    std::size_t components = meas.size() / 3;
    MatrixXd squared_distances(state.cols(), components);

    // Reshape measurement as a matrix
    Map<const MatrixXd> meas_matrix(meas.data(), 3, components);

    // Process all the states
    MatrixXd meas_body(3, components * state.cols());
    std::vector<Transform<double, 3, Eigen::Affine>> poses(state.cols());
    #pragma omp parallel for
    for (std::size_t i = 0; i < state.cols(); i++)
    {
        // Compose translation
        poses[i] = Translation<double, 3>(state.col(i).segment(0, 3));

        // Compose rotation
        poses[i].rotate(AngleAxis<double>(state(9,  i), Vector3d::UnitZ()));
        poses[i].rotate(AngleAxis<double>(state(10, i), Vector3d::UnitY()));
        poses[i].rotate(AngleAxis<double>(state(11, i), Vector3d::UnitX()));

        // Express measurement in body fixed frame
        meas_body.middleCols(components * i, components) = poses[i].inverse() * meas_matrix.colwise().homogeneous();
    }

    // Eval distances
    #pragma omp parallel for collapse(2)
    for (std::size_t i = 0; i < state.cols(); i++)
    {
        for (std::size_t j = 0; j < components; j++)
        {
            const Ref<const Vector3d> meas_j = meas_body.middleCols(components * i, components).col(j);

            std::size_t ret_index;
            double out_dist_sqr;
            KNNResultSet<double> resultSet(1);
            resultSet.init(&ret_index, &out_dist_sqr);
            // Querying tree_ is thread safe as per this issue
            // https://github.com/jlblancoc/nanoflann/issues/54
            tree_->findNeighbors(resultSet, meas_j.data(), nanoflann::SearchParams(10));

            squared_distances(i, j) = out_dist_sqr;
        }
    }

    return std::make_pair(true, squared_distances);
}

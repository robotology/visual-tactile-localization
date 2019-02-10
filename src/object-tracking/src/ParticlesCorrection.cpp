#include <Eigen/Cholesky>

#include <ParticlesCorrection.h>

#include <exception>

#ifdef _OPENMP
#include <omp.h>
#endif

using namespace bfl;
using namespace Eigen;


ParticlesCorrection::ParticlesCorrection
(
    std::unique_ptr<Correction> gauss_corr,
    std::unique_ptr<ProximityLikelihood> lik_model/*,
    std::unique_ptr<StateModel> state_model*/
) noexcept :
    ParticlesCorrection(std::move(gauss_corr), std::move(lik_model)/*, std::move(state_model)*/, 1)
{ }


ParticlesCorrection::ParticlesCorrection
(
    std::unique_ptr<Correction> gauss_corr,
    std::unique_ptr<ProximityLikelihood> lik_model/*,
    std::unique_ptr<StateModel> state_model*/,
    unsigned int seed
) noexcept :
    gaussian_correction_(std::move(gauss_corr)),
    likelihood_model_(std::move(lik_model)),
    // state_model_(std::move(state_model)),
    generator_(std::mt19937_64(seed)),
    distribution_(std::normal_distribution<double>(0.0, 1.0)),
    gaussian_random_sample_([&] { return (distribution_)(generator_); })
{ }


ParticlesCorrection::ParticlesCorrection(ParticlesCorrection&& particles_correction) noexcept :
    PFCorrection(std::move(particles_correction)),
    gaussian_correction_(std::move(particles_correction.gaussian_correction_)),
    likelihood_model_(std::move(particles_correction.likelihood_model_)),
    // state_model_(std::move(particles_correction.state_model_)),
    generator_(std::move(particles_correction.generator_)),
    distribution_(std::move(particles_correction.distribution_)),
    gaussian_random_sample_(std::move(particles_correction.gaussian_random_sample_)) { }


ParticlesCorrection::~ParticlesCorrection() noexcept
{ }


void ParticlesCorrection::setLikelihoodModel(std::unique_ptr<LikelihoodModel> likelihood_model)
{
    throw std::runtime_error("ERROR::PARTICLESCORRECTION::SETLIKELIHOODMODEL\nERROR:\n\tCall to unimplemented base class method.");
}


void ParticlesCorrection::setMeasurementModel(std::unique_ptr<MeasurementModel> measurement_model)
{
    throw std::runtime_error("ERROR::PARTICLESCORRECTION::SETMEASUREMENTMODEL\nERROR:\n\tCall to unimplemented base class method.");
}


MeasurementModel& ParticlesCorrection::getMeasurementModel()
{
    return gaussian_correction_->getMeasurementModel();
}


LikelihoodModel& ParticlesCorrection::getLikelihoodModel()
{
    throw std::runtime_error("ERROR::PARTICLESCORRECTION::GETLIKELIHOODMODEL\nERROR:\n\tCall to unimplemented base class method.");
}


std::pair<bool, Eigen::VectorXd> ParticlesCorrection::getLikelihood()
{
    return std::make_pair(valid_likelihood_, likelihood_);
}


void ParticlesCorrection::reset()
{
    gaussian_correction_->reset();
}

void ParticlesCorrection::correctStep(const bfl::ParticleSet& pred_particles, bfl::ParticleSet& corr_particles)
{
    /* Propagate Gaussian belief associated to each particle. */
    gaussian_correction_->correctStep(pred_particles, corr_particles);

    /* Sample from the proposal distribution. */
    #pragma omp parallel for
    for (std::size_t i = 0; i < pred_particles.components; i++)
    {
        corr_particles.state(i) = sampleFromProposal(corr_particles.mean(i), corr_particles.covariance(i));
    }

    /* Evaluate the likelihood. */
    std::tie(valid_likelihood_, likelihood_) = likelihood_model_->likelihood(getMeasurementModel(), corr_particles.state());

    if (!valid_likelihood_)
    {
        corr_particles = pred_particles;

        return;
    }

    /* Update weights in the log space. */
    corr_particles.weight() = pred_particles.weight() + likelihood_;
    // for (std::size_t i = 0; i < pred_particles.components; i++)
    //     corr_particles.weight(i) = pred_particles.weight(i) + likelihood_(i) -
    //         evaluateProposal(corr_particles.state(i), corr_particles.mean(i), corr_particles.covariance(i));
}


Eigen::VectorXd ParticlesCorrection::sampleFromProposal(const Eigen::VectorXd& mean, const Eigen::MatrixXd& covariance)
{
    /* Extract only the part of the mean relative to the pose and discard velocities. */
    VectorXd mean_pos(6);
    mean_pos.head<3>() = mean.head<3>();
    mean_pos.tail<3>() = mean.tail<3>();

    /* Extract only the part of the covariance matrix relative to the pose and discard velocities. */
    MatrixXd covariance_pos(6, 6);
    covariance_pos.block<3, 3>(0, 0) = covariance.block<3, 3>(0, 0);
    covariance_pos.block<3, 3>(3, 3) = covariance.block<3, 3>(9, 9);
    covariance_pos.block<3, 3>(0, 3) = covariance.block<3, 3>(0, 9);
    covariance_pos.block<3, 3>(3, 0) = covariance.block<3, 3>(9, 0);

    /* Evaluate the square root of the state covariance matrix using the LDL' decomposition
       (it can be used even if the covariance matrix is positive semidefinite). */
    LDLT<MatrixXd> chol_ldlt(covariance_pos);
    MatrixXd sqrt_P = (chol_ldlt.transpositionsP() * MatrixXd::Identity(mean_pos.size(), mean_pos.size())).transpose() *
                       chol_ldlt.matrixL() *
                       chol_ldlt.vectorD().real().cwiseSqrt().asDiagonal();

    /* Sample i.i.d standard normal univariates. */
    VectorXd rand_vectors(mean_pos.size());
    for (int i = 0; i < rand_vectors.size(); i++)
        rand_vectors(i) = gaussian_random_sample_();

    /* Evaluate a sample from a normal multivariate having mean `mean_pos` and covariance 'covariance_pos'. */
    VectorXd sample_pos = mean_pos + sqrt_P * rand_vectors;

    // Handle angular components of the state
    sample_pos.tail<3>() = (std::complex<double>(0.0,1.0) * sample_pos.tail<3>()).array().exp().arg();

    /* Copy the velocities. */
    VectorXd sample(mean.size());
    sample.head<3>() = sample_pos.head<3>();
    sample.segment<6>(3) = mean.segment<6>(3);
    sample.tail<3>() = sample_pos.tail<3>();

    return sample;
}


double ParticlesCorrection::evaluateProposal
(
    const Eigen::VectorXd& state,
    const Eigen::VectorXd& mean,
    const Eigen::MatrixXd& covariance
)
{
    /* Evaluate the proposal distribution, a Gaussian centered in 'mean' and having
       covariance 'covariance', in the state 'state'. */
    VectorXd difference = state - mean;

    return (-0.5 * static_cast<double>(difference.size()) * log(2.0 * M_PI) -0.5 * log(covariance.determinant()) -0.5 * (difference.transpose() * covariance.inverse() * difference).array()).coeff(0);
}

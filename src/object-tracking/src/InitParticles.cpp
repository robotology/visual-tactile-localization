#include <InitParticles.h>

#include <cmath>

using namespace bfl;
using namespace Eigen;

InitParticles::InitParticles
(
    const Ref<const VectorXd>& center,
    const Ref<const VectorXd>& radius,
    const Ref<const MatrixXd>& initial_covariance
) :
    InitParticles(1, center, radius, initial_covariance)
{ }


InitParticles::InitParticles
(
    const unsigned int seed,
    const Ref<const VectorXd>& center,
    const Ref<const VectorXd>& radius,
    const Ref<const MatrixXd>& initial_covariance
) :
    initial_covariance_(initial_covariance),
    generator_        (std::mt19937_64(seed)),
    uniform_x_        (std::uniform_real_distribution<double>(center(0) - radius(0), center(0) + radius(0))),
    uniform_y_        (std::uniform_real_distribution<double>(center(1) - radius(1), center(1) + radius(1))),
    uniform_z_        (std::uniform_real_distribution<double>(center(2) - radius(2), center(2) + radius(2))),
    uniform_yaw_      (std::uniform_real_distribution<double>(center(3) - radius(3) / 2.0, center(3) + radius(3) / 2.0)),
    uniform_pitch_    (std::uniform_real_distribution<double>(center(4) - radius(4) / 2.0, center(4) + radius(4) / 2.0)),
    uniform_roll_     (std::uniform_real_distribution<double>(center(5) - radius(5) / 2.0, center(5) + radius(5) / 2.0)),
    uniform_gen_x_    ([&] { return (uniform_x_)(generator_); }),
    uniform_gen_y_    ([&] { return (uniform_y_)(generator_); }),
    uniform_gen_z_    ([&] { return (uniform_z_)(generator_); }),
    uniform_gen_yaw_  ([&] { return (uniform_yaw_)(generator_); }),
    uniform_gen_pitch_([&] { return (uniform_pitch_)(generator_); }),
    uniform_gen_roll_ ([&] { return (uniform_roll_)(generator_); })
{ }

#include <iostream>
bool InitParticles::initialize(ParticleSet& particles)
{
    for (int i = 0; i < particles.state().cols(); ++i)
    {
        // Initialize mean state with zero velocities
        VectorXd random_state = VectorXd::Zero(12);
        random_state(0) = uniform_gen_x_();
        random_state(1) = uniform_gen_y_();
        random_state(2) = uniform_gen_z_();
        random_state(9) = uniform_gen_yaw_();
        random_state(10) = uniform_gen_pitch_();
        random_state(11) = uniform_gen_roll_();

        particles.mean(i) = random_state;

        // Initialize state covariance
        particles.covariance(i) = initial_covariance_;
    }

    // Set particles position the same as the mean state
    // TODO: they should be sampled from the initial gaussian
    particles.state() = particles.mean();

    // Initialize weights
    particles.weight().fill(-std::log(particles.state().cols()));

    return true;
}

#include <DiscretizedKinematicModelTDD.h>

#include <BayesFilters/utils.h>

#include <Eigen/Dense>

#include <iostream>

using namespace Eigen;


struct DiscretizedKinematicModelTDD::ImplData
{
    enum class Modality
    {
        Iteration,
        Time
    };

    Modality modality_;

    unsigned int iterations_;

    unsigned int current_iterations_ = 0;

    double seconds_;

    double current_seconds_ = 0.0;

    double gain_;

    bfl::utils::CpuTimer<> timer_;
};


DiscretizedKinematicModelTDD::DiscretizedKinematicModelTDD
(
    const double sigma_x,  const double sigma_y,  const double sigma_z,
    const double sigma_yaw, const double sigma_pitch, const double sigma_roll,
    const double gain, const int max_iterations
) :
    DiscretizedKinematicModelTDD(sigma_x, sigma_y, sigma_z, sigma_yaw, sigma_pitch, sigma_roll, gain)
{
    ImplData& rImpl = *pImpl_;


    rImpl.modality_ = ImplData::Modality::Iteration;

    rImpl.iterations_ = max_iterations;

    rImpl.seconds_ = std::numeric_limits<double>::infinity();
}


DiscretizedKinematicModelTDD::DiscretizedKinematicModelTDD
(
    const double sigma_x,  const double sigma_y,  const double sigma_z,
    const double sigma_yaw, const double sigma_pitch, const double sigma_roll,
    const double gain, double max_seconds
) :
    DiscretizedKinematicModelTDD(sigma_x, sigma_y, sigma_z, sigma_yaw, sigma_pitch, sigma_roll, gain)
{
    ImplData& rImpl = *pImpl_;


    rImpl.modality_ = ImplData::Modality::Time;

    rImpl.iterations_ = std::numeric_limits<int>::infinity();

    rImpl.seconds_ = std::abs(max_seconds);

    if (max_seconds < 0)
    {
        std::cerr << "WARNING::DISCRETIZEDKINEMATICMODELTDD::CTOR\n";
        std::cerr << "WARNING::LOG:\n\tInput parameter `max_seconds` is negative. Used as positive.\n";
        std::cerr << "WARNING::LOG:\n\tProvided: " << max_seconds << ". Used " << rImpl.seconds_ << "." << std::endl;
    }
}


DiscretizedKinematicModelTDD::DiscretizedKinematicModelTDD
(
    const double sigma_x,  const double sigma_y,  const double sigma_z,
    const double sigma_yaw, const double sigma_pitch, const double sigma_roll,
    const double gain
) :
    pImpl_(std::unique_ptr<ImplData>(new ImplData))
{
    sigma_position_.resize(3);
    sigma_position_ << sigma_x, sigma_y, sigma_z;

    sigma_orientation_.resize(3);
    sigma_orientation_ << sigma_yaw, sigma_pitch, sigma_roll;

    F_.resize(12, 12);
    F_ = MatrixXd::Zero(12, 12);

    Q_.resize(12, 12);
    Q_ = MatrixXd::Zero(12, 12);

    // Evaluate F and Q matrices using a default
    // sampling time to be updated online
    evaluateStateTransitionMatrix(0.01);
    evaluateNoiseCovarianceMatrix(0.01);

    pImpl_->gain_ = gain;
}


DiscretizedKinematicModelTDD::~DiscretizedKinematicModelTDD()
{ }


void DiscretizedKinematicModelTDD::evaluateStateTransitionMatrix(const double T)
{
    // Compute the state transition matrix
    F_.block<3, 3>(0, 0) = Matrix3d::Identity();
    F_.block<3, 3>(0, 3) = T * Matrix3d::Identity();
    F_.block<3, 3>(3, 3) = Matrix3d::Identity();
    F_.block<3, 3>(6, 6) = Matrix3d::Identity();
    F_.block<3, 3>(9, 6) = T * Matrix3d::Identity();
    F_.block<3, 3>(9, 9) = Matrix3d::Identity();
}


void DiscretizedKinematicModelTDD::evaluateNoiseCovarianceMatrix(const double T)
{
    // Compose noise covariance matrix for the linear acceleration part
    MatrixXd Q_pos(6, 6);
    Q_pos.block<3, 3>(0, 0) = sigma_position_.asDiagonal() * (std::pow(T, 3.0) / 3.0);
    Q_pos.block<3, 3>(0, 3) = sigma_position_.asDiagonal() * (std::pow(T, 2.0) / 2.0);
    Q_pos.block<3, 3>(3, 0) = sigma_position_.asDiagonal() * (std::pow(T, 2.0) / 2.0);
    Q_pos.block<3, 3>(3, 3) = sigma_position_.asDiagonal() * T;

    // Compose noise covariance matrix for the euler angle rates part
    MatrixXd Q_ang(6, 6);
    Q_ang.block<3, 3>(0, 0) = sigma_orientation_.asDiagonal() * T;
    Q_ang.block<3, 3>(0, 3) = sigma_orientation_.asDiagonal() * (std::pow(T, 2.0) / 2.0);
    Q_ang.block<3, 3>(3, 0) = sigma_orientation_.asDiagonal() * (std::pow(T, 2.0) / 2.0);
    Q_ang.block<3, 3>(3, 3) = sigma_orientation_.asDiagonal() * (std::pow(T, 3.0) / 3.0);

    Q_.block<6, 6>(0, 0) = Q_pos;
    Q_.block<6, 6>(6, 6) = Q_ang;
}

std::pair<std::size_t, std::size_t> DiscretizedKinematicModelTDD::getOutputSize() const
{
    // 9 linear components (x, y, z, x_dot, y_dot, z_dot, yaw_dot, pitch_dot, roll_dot)
    // 3 angular components (yaw, pitch, roll)
    return std::make_pair(9, 3);
}


Eigen::MatrixXd DiscretizedKinematicModelTDD::getStateTransitionMatrix()
{
    return F_;
}


Eigen::MatrixXd DiscretizedKinematicModelTDD::getNoiseCovarianceMatrix()
{
    ImplData& rImpl = *pImpl_;


    double damper = 1.0;

    switch (rImpl.modality_)
    {
        case ImplData::Modality::Iteration:
        {
            damper = (rImpl.current_iterations_ <= rImpl.iterations_) ? std::exp(-rImpl.current_iterations_ / rImpl.gain_) : 0.0;

            break;
        }

        case ImplData::Modality::Time:
        {
            std::cout << rImpl.current_seconds_ << std::endl;

            damper = (rImpl.current_seconds_ <= rImpl.seconds_) ? std::exp(-rImpl.current_seconds_ / rImpl.gain_) : 0.0;

            break;
        }

        default:
            return Q_;
    }

    std::cout << damper << std::endl;

    return damper * Q_;
}

bool DiscretizedKinematicModelTDD::setProperty(const std::string& property)
{
    ImplData& rImpl = *pImpl_;


    if (property == "tick")
    {
        // Evaluate elapsed time and reset matrices F_ and Q_
        std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();

        if (last_time_set_)
        {
            std::chrono::duration<double, std::milli> delta_chrono = now - last_time_;
            double delta = delta_chrono.count() / 1000.0;

            // Avoid too large delta during playback
            // if the user stops streaming of data
            if (delta > 0.3)
                delta = 0.01;

            evaluateStateTransitionMatrix(delta);
            evaluateNoiseCovarianceMatrix(delta);
        }

        last_time_ = now;
        last_time_set_ = true;

        return true;
    }
    else if (property == "reset_time")
    {
        last_time_set_ = false;

        return true;
    }
    else if (property == "tdd_reset")
    {
        switch (rImpl.modality_)
        {
        case ImplData::Modality::Iteration:
        {
            rImpl.current_iterations_ = 0;

            break;
        }

        case ImplData::Modality::Time:
        {
            rImpl.timer_.stop();

            break;
        }

        default:
            return false;
        }

        return true;
    }
    else if (property == "tdd_advance")
    {
        switch (rImpl.modality_)
        {

        case ImplData::Modality::Iteration:
        {
            ++rImpl.current_iterations_;

            break;
        }

        case ImplData::Modality::Time:
        {
            rImpl.current_seconds_ = rImpl.timer_.elapsed() / 1000.0;

            if (!rImpl.timer_.is_running())
                rImpl.timer_.start();

            break;
        }

        default:
            return false;
        }

        return true;
    }

    return false;
}

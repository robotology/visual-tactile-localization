#ifndef RANDOM3DPOSE_H
#define RANDOM3DPOSE_H

#include <BayesFilters/StateModel.h>

#include <functional>
#include <random>


class Random3DPose : public bfl::StateModel
{
public:
    Random3DPose
        (
            /**
             * Sampling time
             */
            const double T,
            /**
             * Power Spectral Density of linear acceleration noise
             */
            const double sigma_x,  const double sigma_y,  const double sigma_z,
            /**
             * Power Spectral Density of angular acceleration noise
             */
            const double sigma_wx, const double sigma_wy, const double sigma_wz,
            /**
             * Seed for random numbers generator
             */
            unsigned int seed
        ) noexcept;

    Random3DPose
        (
            const double T,
            const double sigma_x,
            const double sigma_y,
            const double sigma_z,
            const double sigma_wx,
            const double sigma_wy,
            const double sigma_wz
        ) noexcept;

    virtual ~Random3DPose() noexcept { };

    void motion
    (
        const Eigen::Ref<const Eigen::MatrixXd>& cur_states,
        Eigen::Ref<Eigen::MatrixXd> prop_states
    ) override;

    void propagate
    (
        const Eigen::Ref<const Eigen::MatrixXd>& cur_states,
        Eigen::Ref<Eigen::MatrixXd> prop_states
    ) override;

    bool setProperty(const std::string& property) override { return false; };

    std::pair<std::size_t, std::size_t> getOutputSize() const override;

protected:
    Eigen::MatrixXd getNoiseSample(const std::size_t num);

   /**
     * Sampling interval in (s).
     */
    double T_;

    /**
     * Square root covariance matrix of awgn for the linear part.
     */
    Eigen::Matrix<double, 6, 6> sqrt_Q_pos_;

    /**
     * Square root covariance matrix of awgn for the angular part.
     */
    Eigen::Matrix3d sqrt_Q_ang_;

    /**
     * A call to `gauss_rnd_sample_()` returns i.i.d normal univariates.
     */
    std::function<double()> gauss_rnd_sample_;

    std::mt19937_64 generator_;

    std::normal_distribution<double> distribution_;
};

#endif /* RANDOM3DPOSE_H */

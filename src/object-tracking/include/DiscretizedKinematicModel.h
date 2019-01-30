#ifndef DISCRETIZEDKINEMATICMODEL_H
#define DISCRETIZEDKINEMATICMODEL_H

#include <BayesFilters/LinearStateModel.h>

#include <Eigen/Dense>

#include <chrono>

class DiscretizedKinematicModel : public bfl::LinearStateModel
{
public:
    DiscretizedKinematicModel
    (
        const double sigma_x,
        const double sigma_y,
        const double sigma_z,
        const double sigma_phi,
        const double sigma_theta,
        const double sigma_psi
    );

    virtual ~DiscretizedKinematicModel();

    Eigen::MatrixXd getStateTransitionMatrix() override;

    bool setProperty(const std::string& property) override;

    Eigen::MatrixXd getNoiseCovarianceMatrix();

    std::pair<std::size_t, std::size_t> getOutputSize() const override;

protected:
    void evaluateStateTransitionMatrix(const double T);

    void evaluateNoiseCovarianceMatrix(const double T);

    /**
     * State transition matrix.
     */
    Eigen::MatrixXd F_;

    /**
     * Noise covariance matrix.
     */
    Eigen::MatrixXd Q_;

    /**
     * Squared power spectral densities
     */
    Eigen::VectorXd sigma_position_;

    Eigen::VectorXd sigma_orientation_;

    std::chrono::high_resolution_clock::time_point last_time_;

    bool last_time_set_ = false;
};

#endif /* DISCRETIZEDKINEMATICMODEL_H */

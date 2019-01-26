#ifndef DISCRETIZEDKINEMATICMODEL_H
#define DISCRETIZEDKINEMATICMODEL_H

#include <BayesFilters/LinearStateModel.h>

#include <Eigen/Dense>

class DiscretizedKinematicModel : public bfl::LinearStateModel
{
public:
    DiscretizedKinematicModel
    (
        const double T,
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
    /**
     * State transition matrix.
     */
    Eigen::MatrixXd F_;

    /**
     * Noise covariance matrix.
     */
    Eigen::MatrixXd Q_;
};

#endif /* DISCRETIZEDKINEMATICMODEL_H */

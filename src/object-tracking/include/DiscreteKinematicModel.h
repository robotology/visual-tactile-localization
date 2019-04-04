/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef DISCRETEKINEMATICMODEL_H
#define DISCRETEKINEMATICMODEL_H

#include <BayesFilters/LinearStateModel.h>

#include <Eigen/Dense>

class DiscreteKinematicModel : public bfl::LinearStateModel
{
public:
    DiscreteKinematicModel
    (
        const double T,
        const double q_x,
        const double q_y,
        const double q_z,
        const double q_x_dot,
        const double q_y_dot,
        const double q_z_dot,
        const double q_yaw,
        const double q_pitch,
        const double q_roll,
        const double q_yaw_dot,
        const double q_pitch_dot,
        const double q_roll_dot

    );

    virtual ~DiscreteKinematicModel();

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

#endif /* DISCRETEKINEMATICMODEL_H */

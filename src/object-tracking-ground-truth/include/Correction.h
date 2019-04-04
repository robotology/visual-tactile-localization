/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef CORRECTION_H
#define CORRECTION_H

#include <BayesFilters/LinearMeasurementModel.h>
#include <BayesFilters/KFCorrection.h>

#include <Eigen/Dense>


class Correction : public bfl::KFCorrection
{
public:
    Correction(std::unique_ptr<bfl::LinearMeasurementModel> meas_model) noexcept;

    virtual ~Correction();

    void correctStep(const bfl::GaussianMixture& pred_state, bfl::GaussianMixture& corr_state) override;
};

#endif /* CORRECTION_H */

/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <Correction.h>

#include <cmath>

using namespace bfl;
using namespace Eigen;


Correction::Correction(std::unique_ptr<LinearMeasurementModel> meas_model) noexcept :
    KFCorrection(std::move(meas_model))
{ }


Correction::~Correction()
{ }


void Correction::correctStep(const GaussianMixture& pred_state, GaussianMixture& corr_state)
{
    KFCorrection::correctStep(pred_state, corr_state);

    // Handle angular components of the state
    corr_state.mean().bottomRows(3) = (std::complex<double>(0.0,1.0) * corr_state.mean().bottomRows(3)).array().exp().arg();
}

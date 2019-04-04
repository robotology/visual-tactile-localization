/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <GaussianFilter_.h>

using namespace bfl;


GaussianFilter_::GaussianFilter_
(
    Gaussian& initial_state,
    std::unique_ptr<GaussianPrediction> prediction,
    std::unique_ptr<Correction> correction
) noexcept :
    predicted_state_(initial_state.dim_linear, initial_state.dim_circular),
    corrected_state_(initial_state),
    prediction_(std::move(prediction)),
    correction_(std::move(correction))
{ }


GaussianFilter_::GaussianFilter_(GaussianFilter_&& gf) noexcept :
    predicted_state_(std::move(gf.predicted_state_)),
    corrected_state_(std::move(gf.corrected_state_)),
    prediction_(std::move(gf.prediction_)),
    correction_(std::move(gf.correction_))
{ }


GaussianFilter_::~GaussianFilter_() noexcept
{ }


bool GaussianFilter_::initialization()
{
    return true;
}


void GaussianFilter_::filteringStep()
{
    prediction_->predict(corrected_state_, predicted_state_);
    correction_->correct(predicted_state_, corrected_state_);

    log();
}


bool GaussianFilter_::runCondition()
{
    return true;
}


bool GaussianFilter_::skip(const std::string& what_step, const bool status)
{
    if (what_step == "prediction" ||
        what_step == "state"      ||
        what_step == "exogenous")
        return prediction_->skip(what_step, status);

    if (what_step == "correction")
        return correction_->skip(status);

    if (what_step == "all")
    {
        bool return_status = true;

        return_status &= prediction_->skip("prediction", status);
        return_status &= prediction_->skip("state", status);
        return_status &= prediction_->skip("exogenous", status);

        return_status &= correction_->skip(status);

        return return_status;
    }

    return false;
}

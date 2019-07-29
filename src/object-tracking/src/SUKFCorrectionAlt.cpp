/*
 * Copyright (C) 2016-2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * BSD 3-Clause license. See the accompanying LICENSE file for details.
 */

#include <BayesFilters/SUKFCorrection.h>
#include <BayesFilters/directional_statistics.h>
#include <BayesFilters/utils.h>

#include <opencv2/opencv.hpp>

#include <unordered_map>

using namespace bfl;
using namespace bfl::directional_statistics;
using namespace bfl::sigma_point;
using namespace Eigen;


SUKFCorrection::SUKFCorrection
(
    std::unique_ptr<AdditiveMeasurementModel> measurement_model,
    const size_t n,
    const double alpha,
    const double beta,
    const double kappa,
    const size_t measurement_sub_size,
    const bool use_reduced_noise_covariance_matrix
) noexcept :
    measurement_model_(std::move(measurement_model)),
    ut_weight_(n, alpha, beta, kappa),
    measurement_sub_size_(measurement_sub_size),
    use_reduced_noise_covariance_matrix_(use_reduced_noise_covariance_matrix)
{ }


SUKFCorrection::SUKFCorrection(SUKFCorrection&& sukf_correction) noexcept :
    measurement_model_(std::move(sukf_correction.measurement_model_)),
    ut_weight_(sukf_correction.ut_weight_),
    measurement_sub_size_(sukf_correction.measurement_sub_size_),
    use_reduced_noise_covariance_matrix_(sukf_correction.use_reduced_noise_covariance_matrix_)
{ }


MeasurementModel& SUKFCorrection::getMeasurementModel()
{
    return *measurement_model_;
}


std::pair<bool, VectorXd> SUKFCorrection::getLikelihood()
{
    /* This method uses the member variable propagated_sigma_points_.
       After each call to correctStep(), that variable contains a matrix Y
       such that the predicted covariance of measurement is S = Y * Y^{T} + R */

    if ((innovations_.rows() == 0) || (innovations_.cols() == 0))
        return std::make_pair(false, VectorXd());

    /* Evaluate the matrix containing the measurement noise covariance for each submeasurement. */
    MatrixXd R(measurement_sub_size_, innovations_.rows());
    for (std::size_t i = 0; i < innovations_.rows() / measurement_sub_size_; i++)
    {
        R.middleCols(i * measurement_sub_size_, measurement_sub_size_) = getNoiseCovarianceMatrix(i);
    }

    /* Evaluate the likelihoods. */
    VectorXd likelihood(innovations_.cols());
    std::size_t size_sigma_points = propagated_sigma_points_.cols() / innovations_.cols();
    for (std::size_t i = 0; i < innovations_.cols(); i++)
    {
        Ref<MatrixXd> Y = propagated_sigma_points_.middleCols(size_sigma_points * i, size_sigma_points);
        likelihood(i) = utils::multivariate_gaussian_density_UVR(innovations_.col(i), VectorXd::Zero(innovations_.rows()), Y, Y.transpose(), R).coeff(0);
    }

    return std::make_pair(true, likelihood);
}


void SUKFCorrection::correctStep(const GaussianMixture& pred_state, GaussianMixture& corr_state)
{
    /* Sample sigma points. */
    MatrixXd input_sigma_points = sigma_point::sigma_point(pred_state, ut_weight_.c);

    /* Propagate sigma points. */
    Data pred;
    bool valid_pred;
    std::tie(valid_pred, pred) = measurement_model_->predictedMeasure(input_sigma_points);

    if (!valid_pred)
    {
        corr_state = pred_state;
        return;
    }

    /* Cast data to MatrixXd. */
    std::tuple<std::size_t, MatrixXd, std::vector<MatrixXd>> prediction_tuple = bfl::any::any_cast<std::tuple<std::size_t, MatrixXd, std::vector<MatrixXd>>>(pred);
    // propagated_sigma_points_ = bfl::any::any_cast<MatrixXd&&>(std::move(pred));
    std::size_t data_size_3d = std::get<0>(prediction_tuple);
    propagated_sigma_points_ = std::get<1>(prediction_tuple);
    std::vector<MatrixXd> segmentation_predictions = std::get<2>(prediction_tuple);


    /* Get the current measurement if available. */
    bool valid_measurement;
    Data measurement;
    std::tie(valid_measurement, measurement) = measurement_model_->measure();
    MatrixXd measurement_eigen = bfl::any::any_cast<MatrixXd&&>(std::move(measurement));

    /* Remove useless points from segmentation predictions. */
    double delta = 0.2;
    Ref<MatrixXd> segmentation_meas = measurement_eigen.bottomRows(measurement_eigen.rows() - data_size_3d * 3);
    // VectorXd segmentation_meas_scaled(segmentation_meas.rows(), 1);
    std::vector<std::vector<cv::Point>> contours(1);
    VectorXd mean = VectorXd::Zero(2);
    for (std::size_t i = 0; i < segmentation_meas.size() / 2; i++)
        mean += segmentation_meas.col(0).segment(i * 2, 2);
    mean /= (segmentation_meas.size() / 2 );
    for (std::size_t i = 0; i < segmentation_meas.size() / 2; i++)
    {
        VectorXd scaled_point = segmentation_meas.col(0).segment(i * 2, 2) + (segmentation_meas.col(0).segment(i * 2, 2) - mean) * delta;
        contours[0].push_back(cv::Point(scaled_point(0), scaled_point(1)));
    }
    for (std::size_t i = 0; i < segmentation_predictions.size(); i++)
    {
        Ref<MatrixXd> segmentation = segmentation_predictions.at(i);
        VectorXi valids = VectorXi::Zero(segmentation.cols());
        for (std::size_t j = 0; j < segmentation.cols(); j++)
        {
            if (cv::pointPolygonTest(contours[0], cv::Point2f(segmentation.col(j)(0), segmentation.col(j)(1)), false) >= 0)
                valids(j) = 1;
        }
        MatrixXd cleaned_segmentation(2, valids.sum());
        // std::cout << "i: " << i << std::endl;
        // std::cout << "size:" << valids.sum() << std::endl;
        for (std::size_t j = 0, k = 0; j < segmentation.cols(); j++)
            if (valids(j) == 1)
            {
                cleaned_segmentation.col(k) = segmentation.col(j);
                k++;
            }

        segmentation_predictions.at(i) = cleaned_segmentation;
    }

    /* Remove useless mask measurements. */
    Map<MatrixXd> segmentation_meas_cols(segmentation_meas.data(), 2, segmentation_meas.size() / 2);
    VectorXi valid_measure = VectorXi::Zero(segmentation_meas_cols.cols());
    for (std::size_t i = 0; i < segmentation_predictions.size(); i++)
    {
        Ref<MatrixXd> segmentation = segmentation_predictions.at(i);

        for (std::size_t j = 0; j < segmentation.cols(); j++)
        {
            std::size_t min_index;
            (segmentation_meas_cols.colwise() - segmentation.col(j)).colwise().norm().minCoeff(&min_index);
            valid_measure(min_index) = 1;
        }
    }

    /* Store valid measurements. */
    VectorXd cleaned_meas(valid_measure.sum() * 2);
    for (std::size_t i = 0, j = 0; i < valid_measure.size(); i++)
    {
        if (valid_measure(i) == 1)
        {
            cleaned_meas.segment(j * 2, 2) = segmentation_meas_cols.col(i);
            j++;
        }
    }

    /* Store valid associations. */
    MatrixXd cleaned_predictions(cleaned_meas.size(), (pred_state.dim * 2) + 1);
    for (std::size_t i = 0; i < cleaned_predictions.cols(); i++)
    {
        if (segmentation_predictions.at(i).cols() == 0)
        {
            cleaned_predictions.col(i) = cleaned_meas;
        }
        else
        {
            for (std::size_t j = 0; j < cleaned_meas.size() / 2; j++)
            {
                std::size_t min_index;
                (segmentation_predictions.at(i).colwise() - cleaned_meas.segment(j * 2, 2)).colwise().norm().minCoeff(&min_index);
                cleaned_predictions.col(i).segment(j * 2, 2) = segmentation_predictions.at(i).col(min_index);
            }
        }
    }

    // std::cout << "*" << std::endl;
    // std::cout << cleaned_meas.transpose() << std::endl;
    // std::cout << cleaned_predictions.transpose() << std::endl;

    std::size_t data_size_2d = cleaned_meas.size() / 2;
    VectorXd point_cloud = measurement_eigen.col(0).head(data_size_3d * 3);

    measurement_eigen.resize(data_size_3d * 3 + data_size_2d * 2, 1);
    measurement_eigen.col(0).head(data_size_3d * 3) = point_cloud;
    measurement_eigen.col(0).tail(data_size_2d * 2) = cleaned_meas;

    MatrixXd propagated_sigmas = propagated_sigma_points_;
    propagated_sigma_points_.resize(data_size_3d * 3 + data_size_2d * 2, NoChange);
    propagated_sigma_points_.topRows(data_size_3d  * 3) = propagated_sigmas;
    propagated_sigma_points_.bottomRows(data_size_2d * 2).swap(cleaned_predictions);

    /* Evaluate the predicted mean. */
    std::size_t size_sigmas = (pred_state.dim * 2) + 1;
    MatrixXd pred_mean(propagated_sigma_points_.rows(), pred_state.components);
    for (size_t i = 0; i < pred_state.components; i++)
    {
        Ref<MatrixXd> prop_sp = propagated_sigma_points_.middleCols(size_sigmas * i, size_sigmas);

        /* Evaluate the mean. */
        pred_mean.col(i).noalias() = prop_sp * ut_weight_.mean;
    }

    /* Check if the size of the measurement is compatible with measurement_sub_size. */
    // std::pair<std::size_t, std::size_t> sizes = measurement_model_->getOutputSize();
    // std::size_t meas_size = sizes.first + sizes.second;
    // valid_measurement &= (((meas_size) % measurement_sub_size_ ) == 0);

    // if (!valid_measurement)
    // {
    //     corr_state = pred_state;
    //     return;
    // }

    /* Evaluate the innovation if possible. */
    bool valid_innovation;
    Data innovation;
    // MatrixXd measurement_3d;
    // measurement_3d = measurement_eigen.col(0).head(data_size_3d * 3);
    std::tie(valid_innovation, innovation) = measurement_model_->innovation(pred_mean, measurement_eigen);

    if (!valid_innovation)
    {
        corr_state = pred_state;
        return;
    }

    /* Cast innovations once for all. */
    innovations_ = any::any_cast<MatrixXd&&>(std::move(innovation));

    /* From now on using equations from the paper:
       Barfoot, T., McManus, C. (2011),
       'A Serial Approach to Handling High-Dimensional Measurements in the Sigma-Point Kalman Filter.',
       Science and Systems VII,
       MIT Press */

    /* Process all the components in the mixture. */
    MatrixXd sqrt_ut_weight = ut_weight_.covariance.array().sqrt().matrix().asDiagonal();
    for (size_t i = 0; i < pred_state.components; i++)
    {
        /* Compose square root of the measurement covariance matrix.
           IV.C.2.b */
        Ref<MatrixXd> Y = propagated_sigma_points_.middleCols(size_sigmas * i, size_sigmas);

        /* Shift w.r.t. the mean. */
        Y.colwise() -= pred_mean.col(i);

        /* Weight using square root of unscented transform weight. */
        Y *= sqrt_ut_weight;

        /* Compose matrix C and vector d by cycling over all the sub-vector of the measurement
           IV.C.2.c */
        MatrixXd C_inv = MatrixXd::Identity(size_sigmas, size_sigmas);
        VectorXd d = VectorXd::Zero(size_sigmas);

        std::size_t j;
        for (j = 0; j < data_size_3d; j++)
        {
            MatrixXd tmp(size_sigmas, measurement_sub_size_);
            tmp.noalias() = Y.middleRows(measurement_sub_size_ *j, measurement_sub_size_).transpose() * getNoiseCovarianceMatrix(j).inverse();

            C_inv += tmp * Y.middleRows(measurement_sub_size_ *j, measurement_sub_size_);

            d += tmp * innovations_.col(i).middleRows(measurement_sub_size_ *j, measurement_sub_size_);
        }
        for (std::size_t k = 0; k < data_size_2d; k++)
        {
            MatrixXd tmp(size_sigmas, measurement_sub_size_);
            tmp.noalias() = Y.middleRows(j * measurement_sub_size_  + 2 * k, 2).transpose() * getNoiseCovarianceMatrix(point_cloud.size() / measurement_sub_size_ + k).inverse();

            C_inv += tmp * Y.middleRows(j * measurement_sub_size_  + 2 * k, 2);

            d += tmp * innovations_.col(i).middleRows(j * measurement_sub_size_  + 2 * k, 2);
        }

        /* Process input sigma points.
           IV.C.3 */
        Ref<MatrixXd> X = input_sigma_points.middleCols(size_sigmas * i, size_sigmas);
        X.topRows(pred_state.dim_linear).colwise() -= pred_state.mean(i).topRows(pred_state.dim_linear);
        X.bottomRows(pred_state.dim_circular) = directional_sub(X.bottomRows(pred_state.dim_circular), pred_state.mean(i).bottomRows(pred_state.dim_circular));
        X *= sqrt_ut_weight;

        C_inv = C_inv.inverse();

        /* Evaluate the filtered mean.
           IV.C.4 */
        corr_state.mean(i) = pred_state.mean(i) + X * C_inv * d;

        /* Evaluate the filtered covariance.
           IV.C.4 */
        corr_state.covariance(i) = X * C_inv * X.transpose();
    }
}


MatrixXd SUKFCorrection::getNoiseCovarianceMatrix(const std::size_t index)
{
    /* Obtain the noise covariance matrix from the measurement model. */
    MatrixXd R;
    std::tie(std::ignore, R) = measurement_model_->getNoiseCovarianceMatrix();

    if (use_reduced_noise_covariance_matrix_)
        return R;
    else
        return R.block(measurement_sub_size_ * index, measurement_sub_size_ * index, measurement_sub_size_, measurement_sub_size_);
}

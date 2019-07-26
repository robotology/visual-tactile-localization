/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <MetricAddS.h>

using namespace Eigen;


MetricAddS::MetricAddS()
{ }


MetricAddS::~MetricAddS()
{ }


double MetricAddS::evaluate(const Eigen::Transform<double, 3, Eigen::Affine>& estimate, const Eigen::Transform<double, 3, Eigen::Affine>& ground_truth, const Eigen::MatrixXd& model)
{
    /* Evaluate model in estimated pose. */
    MatrixXd estimated_model = estimate * model.colwise().homogeneous();

    /* Evaluate model in ground truth pose. */
    MatrixXd ground_truth_model = ground_truth * model.colwise().homogeneous();

    /* Evaluate ADD metric. */
    MatrixXd difference = ground_truth_model - estimated_model;

    return difference.colwise().norm().sum() / difference.cols();
}

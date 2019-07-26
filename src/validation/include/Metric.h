/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef METRIC_H
#define METRIC_H

#include <Eigen/Dense>


class Metric
{
public:
    Metric();

    virtual ~Metric();

    virtual double evaluate(const Eigen::Transform<double, 3, Eigen::Affine>& estimate, const Eigen::Transform<double, 3, Eigen::Affine>& ground_truth, const Eigen::MatrixXd& model) = 0;
};

#endif /* METRIC_H */

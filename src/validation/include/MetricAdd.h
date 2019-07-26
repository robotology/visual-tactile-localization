/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef METRICADD_H
#define METRICADD_H

#include <Eigen/Dense>

#include <Metric.h>


class MetricAdd : public Metric
{
public:
    MetricAdd();

    ~MetricAdd();

    double evaluate(const Eigen::Transform<double, 3, Eigen::Affine>& estimate, const Eigen::Transform<double, 3, Eigen::Affine>& ground_truth, const Eigen::MatrixXd& model) override;
};

#endif /* METRICADD_H */

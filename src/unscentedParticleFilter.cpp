/******************************************************************************
 *                                                                            *
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Giulia Vezzani <giulia.vezzani@iit.it>
 * @author: Nicola Piga <nicolapiga@gmail.com>
 */

// std
#include <string>
#include <fstream>
#include <iostream>

// yarp
#include <yarp/os/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>
#include <yarp/math/RandnScalar.h>
#include <yarp/math/SVD.h>

// CGAL
#include <CGAL/IO/Polyhedron_iostream.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Cholesky>

//
#include "headers/unscentedParticleFilter.h"

typedef CGAL::Aff_transformation_3<K> Affine;

using namespace yarp::math;

bool UnscentedParticleFilter::loadListDouble(yarp::os::ResourceFinder &rf,
                                             const std::string &key,
                                             const int &size,
                                             yarp::sig::Vector &list)
{
    if (rf.find(key).isNull())
        return false;

    yarp::os::Bottle* b = rf.find(key).asList();
    if (b == nullptr)
        return false;

    if (b->size() != size)
        return false;

    list.resize(size);
    for (size_t i=0; i<b->size(); i++)
    {
        yarp::os::Value item_v = b->get(i);
        if (item_v.isNull())
            return false;

        if (!item_v.isDouble())
        {
            list.clear();
            return false;
        }

        list[i] = item_v.asDouble();
    }
    return true;
}

double UnscentedParticleFilter::normalizeAngle(const double& angle)
{
    double wrapped = angle;
    while (wrapped > M_PI)
    {
        wrapped -= 2 * M_PI;
    }

    while (wrapped < -M_PI)
    {
        wrapped += 2* M_PI;
    }

    return wrapped;
}

void UnscentedParticleFilter::initializationUPF()
{
    for(size_t i=0; i<x.size(); i++ )
    {
        // initialize quantities using parameters
        // obtained from the configuration file
        x[i].prev_weights=x[i].weights=1.0/params.N;
        x[i].P_corr=params.P0;
        x[i].P_proj=params.P0;

        // zero matrices
        x[i].P_pred.zero();
    }
}

void UnscentedParticleFilter::initialRandomize()
{
    for (size_t i=0; i<x.size(); i++)
    {
        x[i].x_corr_prev[0]=yarp::math::Rand::scalar(params.center0[0]-params.radius0[0],params.center0[0]+params.radius0[0]);
        x[i].x_corr_prev[1]=yarp::math::Rand::scalar(params.center0[1]-params.radius0[1],params.center0[1]+params.radius0[1]);
        x[i].x_corr_prev[2]=yarp::math::Rand::scalar(params.center0[2]-params.radius0[2],params.center0[2]+params.radius0[2]);
        x[i].x_corr_prev[3]=yarp::math::Rand::scalar(-M_PI / 2.0, M_PI / 2.0);
        x[i].x_corr_prev[4]=yarp::math::Rand::scalar(-M_PI / 2.0, M_PI / 2.0);
        x[i].x_corr_prev[5]=yarp::math::Rand::scalar(-M_PI / 2.0, M_PI / 2.0);
        x[i].x_proj_prev = x[i].x_corr_prev;
    }
}

void UnscentedParticleFilter::resizeParticle(const int &i)
{
    // take the size of the last measurement received
    int p = 3*curr_meas.size();

    x[i].y_pred.resize(p,0.0);
    x[i].Pyy.resize(p,p);
    x[i].Pxy.resize(params.n,p);
    x[i].K.resize(params.n,p);
    x[i].A.resize(p,1);
    x[i].YsigmaPoints_pred.resize(p,2*params.n+1);
}

void UnscentedParticleFilter::initializeUKFMatrix(const int &i)
{
    x[i].Pyy.zero();
    x[i].Pxy.zero();
    x[i].K.zero();
    // x[i].P_pred_aux.zero();
}

void UnscentedParticleFilter::computeSigmaPoints(const int &i)
{
    yarp::sig::Vector aux;

    yarp::sig::Matrix S,U,V,G;
    yarp::sig::Vector s;
    s.resize(params.n,0.0);
    U.resize(params.n,params.n);
    S.resize(params.n,params.n);
    V.resize(params.n,params.n);
    G.resize(params.n,params.n);

    yarp::math::SVD((params.n+params.lambda)*x[i].P_corr,U,s,V);

    for(size_t k=0; k<params.n; k++)
    {
        s[k]=sqrt(s[k]);
    }
    S.diagonal(s);
    G=U*S;

    x[i].XsigmaPoints_corr.setCol(0,x[i].x_proj_prev);

    for(size_t j=1; j<params.n+1; j++)
        x[i].XsigmaPoints_corr.setCol(j,x[i].x_proj_prev+G.getCol(j-1));

    for(size_t j=params.n+1; j<2*params.n+1; j++)
        x[i].XsigmaPoints_corr.setCol(j,x[i].x_proj_prev-G.getCol(j-1-params.n));

    x[i].WsigmaPoints_covariance[0]=params.lambda/(params.n+params.lambda)+1-pow(params.alpha,2.0)+params.beta;
    x[i].WsigmaPoints_average[0]=params.lambda/(params.n+params.lambda);

    for(size_t j=1; j<2*params.n+1; j++)
    {
        x[i].WsigmaPoints_covariance[j]=1/(2*(params.n+params.lambda));
        x[i].WsigmaPoints_average[j]=1/(2*(params.n+params.lambda));
    }
}

yarp::sig::Matrix UnscentedParticleFilter::homogeneousTransform(const yarp::sig::Vector &pose)
{
    yarp::sig::Matrix H(4,4);

    // translational part
    H(0,3)=pose[0];
    H(1,3)=pose[1];
    H(2,3)=pose[2];

    // // attitude part is obtained from Euler ZYZ
    // double phi=pose[3];
    // double theta=pose[4];
    // double psi=pose[5];
    // H(0,0)=cos(phi)*cos(theta)*cos(psi)-sin(phi)*sin(psi);
    // H(0,1)=-cos(phi)*cos(theta)*sin(psi)-sin(phi)*cos(psi);
    // H(0,2)=cos(phi)*sin(theta);
    // H(1,0)=sin(phi)*cos(theta)*cos(psi)+cos(phi)*sin(psi);
    // H(1,1)=-sin(phi)*cos(theta)*sin(psi)+cos(phi)*cos(psi);
    // H(1,2)=sin(phi)*sin(theta);
    // H(2,0)= -sin(theta)*cos(psi);
    // H(2,1)=sin(theta)*sin(psi);
    // H(2,2)=cos(theta);

    // attitude part is obtained from Euler ZYX
    double phi=pose[3];
    double theta=pose[4];
    double psi=pose[5];
    H(0,0)=cos(phi)*cos(theta);
    H(0,1)=cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
    H(0,2)=cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
    H(1,0)=sin(phi)*cos(theta);
    H(1,1)=sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi);
    H(1,2)=sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi);
    H(2,0)=-sin(theta);
    H(2,1)=cos(theta)*sin(psi);
    H(2,2)=cos(theta)*cos(psi);

    return H;
}

yarp::sig::Vector UnscentedParticleFilter::computeY(const int &k, const int &j)
{
    Point y_pred;
    yarp::sig::Vector out;

    ParticleUPF &particle=x[k];

    // evaluate transformation from object fixed frame to robot frame
    yarp::sig::Matrix Ho2r=homogeneousTransform(particle.XsigmaPoints_pred.getCol(j));

    // evalute transformation from robot frame to object fixed rame
    yarp::sig::Matrix Hr2o=SE3inv(Ho2r);

    // take last measurements received
    out.resize(3*curr_meas.size(),0.0);

    // evaluate measurement equation
    for (int j=0; j<curr_meas.size(); j++)
    {
        Point &p=curr_meas[j];

        double x=Hr2o(0,0)*p[0]+Hr2o(0,1)*p[1]+Hr2o(0,2)*p[2]+Hr2o(0,3);
        double y=Hr2o(1,0)*p[0]+Hr2o(1,1)*p[1]+Hr2o(1,2)*p[2]+Hr2o(1,3);
        double z=Hr2o(2,0)*p[0]+Hr2o(2,1)*p[1]+Hr2o(2,2)*p[2]+Hr2o(2,3);

        y_pred=tree.closest_point(Point(x,y,z));

        out[j*3]=Ho2r(0,0)*y_pred[0]+Ho2r(0,1)*y_pred[1]+Ho2r(0,2)*y_pred[2]+Ho2r(0,3);
        out[j*3+1]=Ho2r(1,0)*y_pred[0]+Ho2r(1,1)*y_pred[1]+Ho2r(1,2)*y_pred[2]+Ho2r(1,3);
        out[j*3+2]=Ho2r(2,0)*y_pred[0]+Ho2r(2,1)*y_pred[1]+Ho2r(2,2)*y_pred[2]+Ho2r(2,3);
    }

    return out;
}

yarp::sig::Vector UnscentedParticleFilter::computeYIdeal(const int &k, const int &j)
{
    Point y_pred;
    yarp::sig::Vector out;

    ParticleUPF &particle=x[k];

    // evaluate attitude of particle
    yarp::sig::Matrix Hm=homogeneousTransform(particle.XsigmaPoints_pred.getCol(j));

    // evaluate real attitude
    yarp::sig::Matrix Hreal=homogeneousTransform(real_pose);

    // evaluate rotational part of the required transformation
    yarp::sig::Matrix H=Hm*Hreal.transposed();
    // override translational part of required transformation
    H(0,3)=x[k].XsigmaPoints_pred(0,j);
    H(1,3)=x[k].XsigmaPoints_pred(1,j);
    H(2,3)=x[k].XsigmaPoints_pred(2,j);

    // take last measurements received
    out.resize(3*curr_meas.size(),0.0);

    // evaluate ideal measurement equation
    for (int j=0; j<curr_meas.size(); j++)
    {
        // evaluate difference between measurement and real pose
        Point &p=curr_meas[j];
        Point diff=Point(p[0]-real_pose[0],
                         p[1]-real_pose[1],
                         p[2]-real_pose[2]);

        out[j*3]=H(0,0)*diff[0]+H(0,1)*diff[1]+H(0,2)*diff[2]+H(0,3);
        out[j*3+1]=H(1,0)*diff[0]+H(1,1)*diff[1]+H(1,2)*diff[2]+H(1,3);
        out[j*3+2]=H(2,0)*diff[0]+H(2,1)*diff[1]+H(2,2)*diff[2]+H(2,3);
    }

    return out;
}

void UnscentedParticleFilter::statePredictionStep(const int &i)
{
    // system model is linear
    // use plain kalman filter prediction here
    x[i].x_pred = x[i].x_proj_prev;
    for(size_t k=0; k<3; k++)
        x[i].x_pred[k] += propagated_input[k];
    x[i].P_pred=x[i].P_corr + params.Q;
}

void UnscentedParticleFilter::measPredictionStep(const int &i)
{
    for(size_t j=0; j<2*params.n+1; j++)
    {
        x[i].XsigmaPoints_pred.setCol(j,x[i].XsigmaPoints_corr.getCol(j));

        //use system input in the prediction
        for(size_t k=0; k<3; k++)
            x[i].XsigmaPoints_pred(k,j) += propagated_input[k];

        if(params.use_ideal_meas_eqn)
        {
            x[i].YsigmaPoints_pred.setCol(j,computeYIdeal(i,j));
        }
        else
        {
            x[i].YsigmaPoints_pred.setCol(j,computeY(i,j));
        }

    }

    x[i].y_pred=x[i].YsigmaPoints_pred*x[i].WsigmaPoints_average;
}

// void UnscentedParticleFilter::computePpred(const int &i)
// {
//     // for(size_t j=0; j<2*params.n+1; j++)
//     // {
//     //     x[i].x_tilde.setCol(0,x[i].XsigmaPoints_pred.getCol(j)-x[i].x_pred);

//     //     x[i].P_pred_aux=x[i].P_pred_aux+x[i].WsigmaPoints_covariance[j]*x[i].x_tilde*x[i].x_tilde.transposed();

//     // }
//     // x[i].P_pred=x[i].P_pred_aux + params.Q_prev;

//     // system model is linear
//     // use plain kalman filter estimate covariance prediction
// }

void UnscentedParticleFilter:: computeCorrectionMatrix(const int &i)
{
    for(size_t j=0; j<2*params.n+1; j++)
    {
        x[i].A.setCol(0,x[i].YsigmaPoints_pred.getCol(j)-x[i].y_pred);

        x[i].Pyy=x[i].Pyy+x[i].WsigmaPoints_covariance[j]*x[i].A*x[i].A.transposed();

        x[i].x_tilde.setCol(0,x[i].XsigmaPoints_pred.getCol(j)-x[i].x_pred);

        x[i].Pxy=x[i].Pxy+x[i].WsigmaPoints_covariance[j]*x[i].x_tilde*x[i].A.transposed();
    }

    //add measurement noise covariance matrix to Pyy
    yarp::sig::Matrix R(3 * curr_meas.size(), 3 * curr_meas.size());
    yarp::sig::Vector diag_R(3 * curr_meas.size(), params.R);
    R.diagonal(diag_R);
    x[i].Pyy = x[i].Pyy + R;
}

void UnscentedParticleFilter::correctionStep(const int &i)
{
    yarp::sig::Vector meas;

    // take last measurements received
    meas.resize(3*curr_meas.size(),0.0);

    for (int j=0; j<curr_meas.size(); j++)
    {
        Point &p=curr_meas[j];

        meas[j*3]=p[0];
        meas[j*3+1]=p[1];
        meas[j*3+2]=p[2];
    }

    yarp::sig::Vector innovation = x[i].K*(meas-x[i].y_pred);
    for (size_t k=3; k<6; k++)
        innovation[k] = normalizeAngle(innovation[k]);

    x[i].x_corr=x[i].x_pred + innovation;

    for (size_t k=3; k<6; k++)
        x[i].x_corr[k] = normalizeAngle(x[i].x_corr[k]);

    x[i].P_corr=x[i].P_pred-x[i].K*x[i].Pyy*x[i].K.transposed();
}

void UnscentedParticleFilter::constrainedUKF(const int &i)
{
    // this function take the a posteriori estimate of the
    // unconstrained UKF and produces a new estimate using
    // PUKF or ECUKF
    // constrained filtering is performed only on the positional part

    // UT parameters
    double beta = 2.0;
    double alpha = 1.0;
    double kappa = 0.0;
    double lambda = pow(alpha, 2.0) * (3 + kappa) - 3;

    // estimated state covariance matrix
    yarp::sig::Matrix P_corr = x[i].P_corr.submatrix(0, 2, 0, 2);

    // estimated state
    yarp::sig::Vector x_corr = x[i].x_corr.subVector(0, 2);

    // sigma points evaluation
    yarp::sig::Vector aux;
    yarp::sig::Matrix S,U,V,G;
    yarp::sig::Vector s;
    s.resize(3,0.0);
    U.resize(3,3);
    S.resize(3,3);
    V.resize(3,3);
    G.resize(3,3);
    yarp::math::SVD((3 + lambda)* P_corr,U,s,V);
    for(size_t k=0; k<params.n; k++)
    {
        s[k]=sqrt(s[k]);
    }
    S.diagonal(s);
    G=U*S;

    yarp::sig::Matrix sigma_points_x(3, 7);

    sigma_points_x.setCol(0, x_corr);
    for(size_t j=1; j<4; j++)
        sigma_points_x.setCol(j, x_corr + G.getCol(j-1));

    for(size_t j=4; j<7; j++)
        sigma_points_x.setCol(j, x_corr - G.getCol(j-1-3));

    yarp::sig::Vector weights_cov(7, 0.0);
    yarp::sig::Vector weights_avg(7, 0.0);
    weights_cov[0] = lambda / (3 + lambda) + 1 - pow(alpha, 2.0) + beta;
    weights_avg[0] = lambda / (3 + lambda);
    for(size_t j=1; j<7; j++)
    {
        weights_cov[j]= 1 / (2 * (3 + lambda));
        weights_avg[j]= 1 / (2 * (3 + lambda));
    }

    // propagation of sigma points through constraints
    yarp::sig::Matrix sigma_points_y(constraints_distances.size(), 7);
    for(size_t j=0; j<7; j++)
    {
        yarp::sig::Vector propagated_sigma_point(constraints_distances.size());
        for (size_t k=0; k<constraints_distances.size(); k++)
        {
            yarp::sig::Vector diff = constraints_positions[k] - sigma_points_x.getCol(j);
            propagated_sigma_point[k] = yarp::math::norm(diff);
        }
        sigma_points_y.setCol(j, propagated_sigma_point);
    }

    // // predicted mean
    yarp::sig::Vector y_pred;
    y_pred = sigma_points_y * weights_avg;

    // predicted covariance
    yarp::sig::Matrix y_diff(constraints_distances.size(), 1);
    yarp::sig::Matrix x_diff(3, 1);
    yarp::sig::Matrix Pyy(constraints_distances.size(), constraints_distances.size());
    yarp::sig::Matrix Pxy(3, constraints_distances.size());
    Pyy.zero();
    Pxy.zero();
    for(size_t j=0; j<7; j++)
    {
        y_diff.setCol(0, sigma_points_y.getCol(j) - y_pred);
        Pyy = Pyy + weights_cov[j] * y_diff * y_diff.transposed();

        x_diff.setCol(0, sigma_points_x.getCol(j) - x_corr);
        Pxy = Pxy + weights_cov[j] * x_diff * y_diff.transposed();
    }

    //add measurement noise covariance matrix to Pyy
    yarp::sig::Vector diag_R(constraints_distances.size(), 1e-4);
    yarp::sig::Matrix R(constraints_distances.size(), constraints_distances.size());
    R.diagonal(diag_R);
    Pyy = Pyy + R;

    // projection
    yarp::sig::Matrix K = Pxy * luinv(Pyy);

    yarp::sig::Vector innovation = K * (constraints_distances - y_pred);
    innovation[2] = 0.0;
    yarp::sig::Vector x_proj = x_corr + innovation;
    yarp::sig::Matrix P_prj = P_corr - K * Pyy * K.transposed();

    x[i].x_proj = x[i].x_corr;
    x[i].x_proj.setSubvector(0, x_proj);
    x[i].x_corr = x[i].x_proj;
    x[i].P_proj = x[i].P_corr;
}

double UnscentedParticleFilter::likelihood(const int &k, double &map_likelihood)
{
    double sum=0.0;
    double likelihood=1.0;
    map_likelihood=1.0;
    int initial_meas;

    ParticleUPF &particle=x[k];

    yarp::sig::Matrix H=homogeneousTransform(particle.x_proj);
    H=SE3inv(H);

    // initialize the squared measurement error for the current measure
    double squared_tot_meas_error = 0.0;

    for (int j=0; j<curr_meas.size(); j++)
    {
        Point &p=curr_meas[j];

        double x=H(0,0)*p[0]+H(0,1)*p[1]+H(0,2)*p[2]+H(0,3);
        double y=H(1,0)*p[0]+H(1,1)*p[1]+H(1,2)*p[2]+H(1,3);
        double z=H(2,0)*p[0]+H(2,1)*p[1]+H(2,2)*p[2]+H(2,3);

        squared_tot_meas_error += tree.squared_distance(Point(x,y,z));
    }

    // standard likelihood
    likelihood=likelihood*exp(-0.5*squared_tot_meas_error/(params.R));

    return likelihood;
}

double UnscentedParticleFilter::multivariateGaussian(const yarp::sig::Vector &x,
                                                     const yarp::sig::Vector &mean,
                                                     const yarp::sig::Matrix &covariance)
{
    double value;

    // eval the difference x - mean
    yarp::sig::Vector diff = x - mean;

    // wrap angular errors
    for (size_t i=3; i<6; i++)
        diff[i] = normalizeAngle(diff[i]);

    // eval the density
    yarp::sig::Matrix diff_m(x.size(), 1);
    diff_m.setCol(0, diff);

    yarp::sig::Matrix tmp(1,1);
    tmp = diff_m.transposed() * yarp::math::luinv(covariance) * diff_m;

    value = exp(-0.5 * tmp(0, 0)) /
        sqrt(pow(2 * M_PI, x.size()) *
             yarp::math::det(covariance));

    return value;
}

double UnscentedParticleFilter::tranProbability(const int &i,
                                                const int &j)
{
    // evaluate transition probability
    yarp::sig::Vector mean = x[j].x_proj_prev;
    mean.setSubvector(0, mean.subVector(0,2) + propagated_input);

    return multivariateGaussian(x[i].x_proj, mean, params.Q);
}

void UnscentedParticleFilter::computeWeights(const int &i, double& sum, const bool &skip_correction)
{
    double lik;
    double map_likelihood;
    double tran_prob;

    if (skip_correction)
    {
        lik = 1.0;
    }
    else
    {
        // evaluate standard likelihood and
        // likelihood corrected for MAP estimate
        lik = likelihood(i,map_likelihood);
    }

    // evaluate transition probability
    tran_prob = tranProbability(i, i);

    // evaluate weights
    x[i].weights=x[i].prev_weights * lik * tran_prob/
        multivariateGaussian(x[i].x_proj, x[i].x_proj, x[i].P_proj);

    sum+=x[i].weights;
}

void UnscentedParticleFilter::normalizeWeights(const int &i,const double &sum, double &sum_squared)
{
    x[i].weights=x[i].weights/sum;
    sum_squared+=pow(x[i].weights,2.0);
}

void UnscentedParticleFilter::resampling()
{
    std::deque<ParticleUPF> new_x;
    yarp::sig::Vector c,u, new_index;

    //Initialization
    c.resize(params.N, 0.0);
    u.resize(params.N, 0.0);
    new_index.resize(params.N, 0);
    new_x.assign(params.N,ParticleUPF());
    c[0]=x[0].weights;

    for(size_t i=1; i<params.N; i++)
    {
        c[i]=c[i-1]+x[i].weights;
    }

    int i=0;
    u[0]=yarp::math::Rand::scalar(0,1)/params.N;

    for(size_t j=0; j<params.N; j++)
    {
        u[j]=u[0]+1.0/params.N*j;

        while(u[j]>c[i])
        {
            i++;
        }

        new_x[j]=x[i];
        new_x[j].weights=1.0/params.N;
    }

    if (params.use_regularization)
    {
        yarp::sig::Vector sample_mean(6, 0.0);
        yarp::sig::Matrix sample_covariance(6, 6);

        // evaluate sample mean before doing effective resampling
        for (size_t i=0; i<x.size(); i++)
            sample_mean += x[i].weights * x[i].x_proj;
        sample_mean[3] = normalizeAngle(sample_mean[3]);
        sample_mean[4] = normalizeAngle(sample_mean[4]);
        sample_mean[5] = normalizeAngle(sample_mean[5]);
        // sample_mean /= x.size();

        // evaluate sample covariance matrix
        sample_covariance.zero();
        yarp::sig::Matrix sample_mean_mat(6, 1);
        sample_mean_mat.setCol(0, sample_mean);
        for (size_t i=0; i<x.size(); i++)
        {
            yarp::sig::Matrix particle_mat(6, 1);
            particle_mat.setCol(0, x[i].x_proj);
            sample_covariance += x[i].weights * (particle_mat - sample_mean_mat) *
                (particle_mat - sample_mean_mat).transposed();
            // sample_covariance /= x.size();
        }

        // perform cholesky decomposition
        Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor> > sample_cov_eig(sample_covariance.data());
        Eigen::LLT<Eigen::Matrix<double, 6, 6> > chol(sample_cov_eig);
        Eigen::Matrix<double, 6, 6> L = chol.matrixL();

        // sample params.N samples from the gaussian distribution
        Eigen::MatrixXd samples(6, params.N);
        samples = gaussian_sampler->samples(params.N);

        for (size_t i=0; i<params.N; i++)
        {
            // generate state increment
            Eigen::Matrix<double, 6, 1> delta = (h_optimal / 2.0) * L * samples.block<6, 1>(0, i);
            yarp::sig::Vector delta_yarp(6, 0.0);
            for (size_t i=0; i<params.n; i++)
                delta_yarp[i] = delta(i, 0);
            new_x[i].x_proj += delta_yarp;
            new_x[i].x_proj[3] = normalizeAngle(new_x[i].x_proj[3]);
            new_x[i].x_proj[4] = normalizeAngle(new_x[i].x_proj[4]);
            new_x[i].x_proj[5] = normalizeAngle(new_x[i].x_proj[5]);
        }
    }

    x=new_x;
}

void UnscentedParticleFilter::selectionStep(const double &sum_squared)
{
    double Neff=1.0/sum_squared;

    // enable resampling only after params.n_steps_before_resampling;
    if (t >= params.n_steps_before_resampling)
    {
        if(Neff < params.n_eff_thr)
        {
            resampling();
        }
    }
    else
    {
        for(size_t j=0;  j<x.size(); j++)
        {
            x[j].weights=1.0/params.N;
        }
    }
}

bool UnscentedParticleFilter::configure(yarp::os::ResourceFinder &rf)
{
    yInfo()<<"UPF configuration";

    // read the number of particles
    params.N=rf.find("N").asInt();
    if (rf.find("N").isNull())
        params.N=rf.check("N",yarp::os::Value(600)).asInt();
    yInfo()<<"UPF Number of particles:"<<params.N;

    // read the center of the initial research region
    params.center0.resize(3,0.0);
    bool check = loadListDouble(rf, "center0", 3, params.center0);
    if(!check)
    {
        params.center0[0]=0.0;
        params.center0[1]=0.0;
        params.center0[2]=0.0;
    }
    yInfo()<<"UPF Center of initial research region:"<<params.center0.toString();

    // read the radius of the initial research region
    params.radius0.resize(3,0.0);
    check = loadListDouble(rf, "radius0", 3, params.radius0);
    if(!check)
    {
        params.radius0[0]=0.4;
        params.radius0[1]=0.4;
        params.radius0[2]=0.4;
    }
    yInfo()<<"UPF Radius of initial research region:"<<params.radius0.toString();

    // read the number of DoFs
    params.n=rf.find("n").asInt();
    if (rf.find("n").isNull())
        params.n=rf.check("n",yarp::os::Value(6)).asInt();
    yInfo()<<"UPF Number of DoF:"<<params.n;

    // read the Neff threshold
    params.n_eff_thr=rf.find("nEffThr").asDouble();
    if (rf.find("nEffThr").isNull())
        params.n_eff_thr=rf.check("nEffThr",yarp::os::Value(10)).asDouble();
    yInfo()<<"UPF Neff threshold:"<<params.n_eff_thr;

    // read the number of steps to wait for
    // before checking Neff and possibly do resampling
    params.n_steps_before_resampling=rf.find("nStepsBefRsmpl").asInt();
    if (rf.find("nStepsBefRsmpl").isNull())
        params.n_steps_before_resampling=rf.check("nStepsBefRsmpl",yarp::os::Value(10)).asInt();
    yInfo()<<"UPF Steps before resamping:"<<params.n_steps_before_resampling;

    // read the parameter beta for the Unscented Transform
    params.beta=rf.find("beta").asDouble();
    if (rf.find("beta").isNull())
        params.beta=rf.check("beta",yarp::os::Value(35.0)).asDouble();
    yInfo()<<"UPF Unscented Transform Beta:"<<params.beta;

    // read the parameter alpha for the Unscented Transform
    params.alpha=rf.find("alpha").asDouble();
    if (rf.find("alpha").isNull())
        params.alpha=rf.check("alpha",yarp::os::Value(1.0)).asDouble();
    yInfo()<<"UPF Unscented Transform Alpha:"<<params.alpha;

    // read the parameter kappa for the Unscented Transform
    params.kappa=rf.find("kappa").asDouble();
    if (rf.find("kappa").isNull())
        params.kappa=rf.check("kappa",yarp::os::Value(2.0)).asDouble();
    yInfo()<<"UPF Unscented Transform Kappa:"<<params.kappa;

    // compute the parameter lambda for the Unscented Transform
    params.lambda=pow(params.alpha,2.0)*(6+params.kappa)-6;
    yInfo()<<"UPF Unscented Transform Lambda:"<<params.lambda;

    // read the ideal measurement equation enabler
    params.use_ideal_meas_eqn=rf.find("useIdealMeasEqn").asBool();
    if (rf.find("useIdealMeasEqn").isNull())
        params.use_ideal_meas_eqn=rf.check("useIdealMeasEqn",yarp::os::Value(false)).asBool();
    yInfo()<<"UPF use ideal measurement equation :"<<params.use_ideal_meas_eqn;

    params.use_regularization=rf.find("useRegularization").asBool();
    if (rf.find("useRegularization").isNull())
        params.use_regularization=rf.check("useRegularization",yarp::os::Value(false)).asBool();
    yInfo()<<"UPF use regularization :"<<params.use_regularization;

    // read the values of the system noise covariance matrix
    yarp::sig::Vector diagQ;
    diagQ.resize(params.n,1);
    check=loadListDouble(rf, "Q0", 6, diagQ);
    if(!check)
    {
        diagQ[0]=0.0001;
        diagQ[1]=0.0001;
        diagQ[2]=0.0001;
        diagQ[3]=0.001;
        diagQ[4]=0.001;
        diagQ[5]=0.001;
    }
    yarp::sig::Matrix Q;
    params.Q.resize(params.n,params.n);
    params.Q.diagonal(diagQ);
    params.Q_prev = params.Q;
    yInfo()<<"UPF Q0:"<<params.Q.toString();

    // read the noise scalar variance R
    params.R=rf.find("R").asDouble();
    if (rf.find("R").isNull())
        params.R=rf.check("R",yarp::os::Value(0.0001)).asDouble();
    yInfo()<<"UPF R (scalar):"<<params.R;

    // read the values of the initial state covariance matrix
    yarp::sig::Vector diagP0;
    diagP0.resize(params.n,1);
    check = loadListDouble(rf, "P0", 6, diagP0);
    if(!check)
    {
        diagP0[0]=0.04;
        diagP0[1]=0.04;
        diagP0[2]=0.04;
        diagP0[3]=3.29;
        diagP0[4]=3.29;
        diagP0[5]=3.29;
    }
    params.P0.resize(params.n,params.n);
    params.P0.diagonal(diagP0);
    yInfo()<<"UPF P0:"<<params.P0.toString();

    // read triangular mesh model filename
    if (!rf.check("modelFile"))
    {
        yError()<<"UnscentedParticleFilter::configure"
                <<"Error: model file not provided!";
        return false;
    }
    std::string modelFileName=rf.findFile("modelFile");

    // read the polyhedron from a .OFF file
    std::ifstream modelFile(modelFileName.c_str());
    if (!modelFile.is_open())
    {
        yError()<<"UnscentedParticleFilter:: configure"
                <<"Error: problem opening model file!";
        return false;
    }
    modelFile>>getModel();
    if (modelFile.bad())
    {
        yError()<<"UscentedParticleFilter::configure"
                <<"Error: problem reading model file!";
        modelFile.close();
        return false;
    }
    modelFile.close();
    yInfo()<<"UPF Model file loaded successfully";

    // optimal scaling factor for regularized particle filter
    h_optimal = pow(4.0 / (params.n + 2), 1.0 / (params.n + 4)) /
        pow(params.N, 1.0 / (params.n + 4));

    // init gaussian sampler required for particle regularization
    Eigen::Matrix<double, 6, 1> mean = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Identity();
    gaussian_sampler = std::unique_ptr<Eigen::EigenMultivariateNormal<double> >(
        new Eigen::EigenMultivariateNormal<double> (mean, cov));

    // init CGAL
    GeometryCGAL::init();

    return true;
}

void UnscentedParticleFilter::init()
{
    // init index of iteration
    t=0;

    // clear system input
    prev_input.resize(3, 0.0);
    new_input.resize(3, 0.0);
    propagated_input.resize(3, 0.0);

    // initialize particles and sample from initial search region
    x.assign(params.N,ParticleUPF());
    initialRandomize();

    // complete UPF initialization
    initializationUPF();

}

void UnscentedParticleFilter::setNewInput(const yarp::sig::Vector &in)
{
    // set the new system input received
    new_input = in;
}

void UnscentedParticleFilter::setNewMeasure(const std::vector<yarp::sig::Vector>& m)
{
    // set the new measure
    curr_meas.clear();
    for(size_t i=0; i<m.size(); i++)
    {
        // each measure is a std::vector of yarp::sig::Vector vectors
        // each Vector is a point
        const yarp::sig::Vector &point = m[i];

        // convert to a CGAL point
        Point p(point[0], point[1], point[2]);
        curr_meas.push_back(p);
    }
}

void UnscentedParticleFilter::setRealPose(const yarp::sig::Vector &pose)
{
    // assign the current real pose
    real_pose = pose;
}

bool UnscentedParticleFilter::setConstraints(const yarp::sig::Vector &sources_distances,
                                             const std::vector<yarp::sig::Vector> &sources_positions)
{
    // the number of distances of the measurement sources
    // have to be the same as the number of positions of the
    // measurement sources
    if (sources_distances.size() != sources_positions.size())
        return false;

    constraints_distances = sources_distances;
    constraints_positions = sources_positions;

    use_constraints = true;
}

void UnscentedParticleFilter::setQ(const yarp::sig::Vector &covariance)
{
    // assign a new covariance matrix
    params.Q.diagonal(covariance);
}

void UnscentedParticleFilter::setAlpha(const double &alpha)
{
    params.alpha = alpha;
}

void UnscentedParticleFilter::setR(const double &variance)
{
    // assign a new variance
    params.R = variance;
}

void UnscentedParticleFilter::resetTime()
{
    t_prev = yarp::os::Time::now();
}

void UnscentedParticleFilter::clearInputs()
{
    propagated_input = 0.0;
    prev_input = 0.0;
}

void UnscentedParticleFilter::skipStep(double &time_stamp)
{
    // evaluate current time
    double t_current = yarp::os::Time::now();
    time_stamp = t_current;
    // evaluate elapsed time
    double t_interval = t_current - t_prev;

    // since this filtering step is skipped
    // the input is propagated
    propagated_input += prev_input * t_interval;

    // update previous value of input
    prev_input = new_input;

    // update previous value of filtering time
    t_prev = t_current;
}

void UnscentedParticleFilter::step(double &time_stamp, const bool &skip_correction)
{
    t++;

    double sum=0.0;
    double sum_squared=0.0;

    // evaluate current time
    double t_current = yarp::os::Time::now();
    time_stamp = t_current;
    // evaluate elapsed time
    double t_interval = t_current - t_prev;

    // update propagated input
    propagated_input += prev_input * t_interval;

    // process all the particles
    for(size_t i=0; i<x.size(); i++ )
    {
        statePredictionStep(i);
        if (skip_correction)
        {
            x[i].x_proj = x[i].x_pred;
            x[i].P_proj = x[i].P_pred;
        }
        else
        {
            resizeParticle(i);
            initializeUKFMatrix(i);
            computeSigmaPoints(i);
            measPredictionStep(i);
            computeCorrectionMatrix(i);
            x[i].K=x[i].Pxy*luinv(x[i].Pyy);
            correctionStep(i);
            x[i].x_proj = x[i].x_corr;
            x[i].P_proj = x[i].P_corr;
        }
        computeWeights(i, sum, skip_correction);
    }

    // normalize weights
    for (size_t i=0; i<x.size(); i++)
    {
        normalizeWeights(i, sum, sum_squared);
    }

    // resampling strategies
    selectionStep(sum_squared);

    // eval estimate
    evalEstimate(skip_correction);

    // update previous value of Q
    params.Q_prev = params.Q;

    // since this filtering step was done
    // reset the propagated input
    // that was accumulated while skipping steps
    propagated_input = 0;

    // update previous value of input
    prev_input = new_input;

    // update previous value of filtering time
    t_prev = t_current;

    // clear constrained filtering
    use_constraints = false;
    constraints_distances.clear();
    constraints_positions.clear();

    for (size_t i=0; i<x.size(); i++)
    {
        // update previous value of x_corr
        x[i].x_corr_prev = x[i].x_corr;

        // update previous vale of x_proj
        x[i].x_proj_prev = x[i].x_proj;

        // update the weights for the next step
        x[i].prev_weights = x[i].weights;
    }
}

yarp::sig::Vector UnscentedParticleFilter::getEstimate()
{
    return current_estimate;
}

void UnscentedParticleFilter::getParticles(std::vector<yarp::sig::Vector> &particles)
{
    for (size_t i=0; i<x.size(); i++)
        particles.push_back(x[i].x_proj);
}

void UnscentedParticleFilter::evalEstimate(const bool &skip_correction)
{
    // extract MAP estimate
    std::deque<double> probability_per_particle;
    double sum_tran_probability;
    double meas_likelihood;
    for(size_t i=0; i<x.size(); i++)
    {
        // sum of transition probabilites
        sum_tran_probability = 0;

        for(size_t j=0; j<x.size(); j++)
            sum_tran_probability += tranProbability(i, j) * x[j].prev_weights;

        // measurements likelihood
        double tmp;
        if (skip_correction)
        {
            meas_likelihood = 1.0;
        }
        else
        {
            meas_likelihood = likelihood(i, tmp);
        }

        // store probability
        probability_per_particle.push_back(meas_likelihood * sum_tran_probability);
    }

    // find arg max
    double max_prob = 0.0;
    int i_max_prob = 0;
    for(size_t i=0; i<x.size(); i++)
    {
        if(max_prob < probability_per_particle[i])
        {
            max_prob = probability_per_particle[i];
            i_max_prob = i;
        }
    }

    current_estimate = x[i_max_prob].x_proj;
}

double UnscentedParticleFilter::evalPerformanceIndex(const yarp::sig::Vector &estimate,
                                                     const std::deque<Point> &points)
{
    yarp::sig::Matrix H=homogeneousTransform(estimate);
    H=SE3inv(H);

    double error_index=0;
    for (size_t i=0; i<points.size(); i++)
    {
        const Point &m=points[i];

        double x=H(0,0)*m[0]+H(0,1)*m[1]+H(0,2)*m[2]+H(0,3);
        double y=H(1,0)*m[0]+H(1,1)*m[1]+H(1,2)*m[2]+H(1,3);
        double z=H(2,0)*m[0]+H(2,1)*m[1]+H(2,2)*m[2]+H(2,3);

        error_index+=sqrt(tree.squared_distance(Point(x,y,z)));
    }

    error_index/=points.size();

    return error_index;
}

void UnscentedParticleFilter::transformObject(const yarp::sig::Vector &estimate,
                                              Polyhedron &transformed)
{
    // eval the homogeneous transformation
    yarp::sig::Matrix H=homogeneousTransform(estimate);
    Affine affine(H(0,0),H(0,1),H(0,2),H(0,3),
                  H(1,0),H(1,1),H(1,2),H(1,3),
                  H(2,0),H(2,1),H(2,2),H(2,3));

    // allocate storage
    transformed = getModel();

    // transform the object model taking into account the estimate
    std::transform(model.points_begin(),model.points_end(),
                   transformed.points_begin(),affine);
}

void UnscentedParticleFilter::getParticleSet(std::deque<ParticleUPF> &set)
{
    set = x;
}
void UnscentedParticleFilter::setParticleSet(const std::deque<ParticleUPF> &set)
{
    x = set;
}


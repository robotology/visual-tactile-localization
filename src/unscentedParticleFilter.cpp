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

// yarp
#include <yarp/os/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>
#include <yarp/math/RandnScalar.h>
#include <yarp/math/SVD.h>

// CGAL
#include <CGAL/IO/Polyhedron_iostream.h>

//
#include "headers/unscentedParticleFilter.h"

typedef CGAL::Aff_transformation_3<K> Affine;

using namespace yarp::math;

bool UnscentedParticleFilter::readCenter(const std::string &tag, yarp::sig::Vector &center0)
{      
    if (yarp::os::Bottle *b=this->rf->find(tag.c_str()).asList())
	if (b->size()>=3)
	{   
	    center0[0]=b->get(0).asDouble();
	    center0[1]=b->get(1).asDouble();
	    center0[2]=b->get(2).asDouble();
	    return true;
	}
    
    return false;
}
    
bool UnscentedParticleFilter::readRadius(const std::string &tag, yarp::sig::Vector &radius0)
{
    if (yarp::os::Bottle *b=this->rf->find(tag.c_str()).asList())
        if (b->size()>=3)
        {
            radius0[0]=b->get(0).asDouble();           
            radius0[1]=b->get(1).asDouble();            
            radius0[2]=b->get(2).asDouble();
				
            return true;
        }
    
    return false;
}
    
bool UnscentedParticleFilter::readDiagonalMatrix(const std::string &tag, yarp::sig::Vector &diag,\
						 const int &dimension)
{
    if (yarp::os::Bottle *b=this->rf->find(tag.c_str()).asList())
        if (b->size()>=dimension)
        {
            for(size_t i; i<dimension; i++)
                diag[i]=b->get(i).asDouble();

            return true;
        }

    return false;
}

void UnscentedParticleFilter::initializationUPF()
{
    for(size_t i=0; i<x.size(); i++ )
    {
	// initialize quantities using parameters
	// obtained from the configuration file
	x[i].prev_weights=x[i].weights=1.0/params.N;
	x[i].P_corr=params.P0;

	// zero matrices
	x[i].P_hat.zero();
	x[i].P_pred.zero();
    }
}

void UnscentedParticleFilter::initialRandomize()
{
    for (size_t i=0; i<x.size(); i++)
    {
        ParticleUPF &particle=x[i];
        
        particle.x_corr[0]=yarp::math::Rand::scalar(params.center0[0]-params.radius0[0],params.center0[0]+params.radius0[0]);
        particle.x_corr[1]=yarp::math::Rand::scalar(params.center0[1]-params.radius0[1],params.center0[1]+params.radius0[1]);
        particle.x_corr[2]=yarp::math::Rand::scalar(params.center0[2]-params.radius0[2],params.center0[2]+params.radius0[2]);
        particle.x_corr[3]=yarp::math::Rand::scalar(0,2*M_PI);
        particle.x_corr[4]=yarp::math::Rand::scalar(0,M_PI);
        particle.x_corr[5]=yarp::math::Rand::scalar(0,2*M_PI);           
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
    x[i].P_pred_aux.zero();
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
    
    x[i].XsigmaPoints_corr.setCol(0,x[i].x_corr_prev);
    
    for(size_t j=1; j<params.n+1; j++)
    {
        x[i].XsigmaPoints_corr.setCol(j,x[i].x_corr_prev+G.getCol(j-1));
	
        x[i].XsigmaPoints_corr(3,j)=fmod(x[i].XsigmaPoints_corr(3,j),2*M_PI);
        x[i].XsigmaPoints_corr(4,j)=fmod(x[i].XsigmaPoints_corr(4,j),2*M_PI);
        x[i].XsigmaPoints_corr(5,j)=fmod(x[i].XsigmaPoints_corr(5,j),2*M_PI);
    }
    
    for(size_t j=params.n+1; j<2*params.n+1; j++)
    {
        x[i].XsigmaPoints_corr.setCol(j,x[i].x_corr_prev-G.getCol(j-1-params.n));
	
        x[i].XsigmaPoints_corr(3,j)=fmod(x[i].XsigmaPoints_corr(3,j),2*M_PI);
        x[i].XsigmaPoints_corr(4,j)=fmod(x[i].XsigmaPoints_corr(4,j),2*M_PI);
        x[i].XsigmaPoints_corr(5,j)=fmod(x[i].XsigmaPoints_corr(5,j),2*M_PI);
    }
    
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

    // attitude part is obtained from Euler ZYZ
    double phi=pose[3];
    double theta=pose[4];
    double psi=pose[5];
    H(0,0)=cos(phi)*cos(theta)*cos(psi)-sin(phi)*sin(psi);
    H(0,1)=-cos(phi)*cos(theta)*sin(psi)-sin(phi)*cos(psi);
    H(0,2)=cos(phi)*sin(theta);
    H(1,0)=sin(phi)*cos(theta)*cos(psi)+cos(phi)*sin(psi);
    H(1,1)=-sin(phi)*cos(theta)*sin(psi)+cos(phi)*cos(psi);
    H(1,2)=sin(phi)*sin(theta);
    H(2,0)= -sin(theta)*cos(psi);
    H(2,1)=sin(theta)*sin(psi);
    H(2,2)=cos(theta);
    
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

void UnscentedParticleFilter::predictionStep(const int &i,
					     yarp::math::RandnScalar &normal_gen)
{
    yarp::sig::Vector random;
    random.resize(params.n,0.0);
    
    yarp::sig::Matrix cholQ;
    cholQ(params.n,params.n);

    for(size_t j=0; j<2*params.n+1; j++)
    {
	for( size_t l=0; l<params.n; l++)
	{		
	    random[l]=normal_gen.RandnScalar::get(0.0, 1.0);
	}
	    
	cholQ=params.Q;

	//since Q is diagonal, chol(Q) is sqrt of its member on diagonal
	    
	for(size_t k=0; k<6; k++)
	{
	    for( size_t l=0; l<params.n; l++)
	    {
		cholQ(k,l)=sqrt(cholQ(k,l));
	    }
	}
	    
	x[i].XsigmaPoints_pred.setCol(j,x[i].XsigmaPoints_corr.getCol(j));//+cholQ*random);

	//use system input in the prediction
	for(size_t k=0; k<3; k++)
	    x[i].XsigmaPoints_pred(k,j) += last_input[k];
	    
	x[i].XsigmaPoints_pred(3,j)=fmod(x[i].XsigmaPoints_pred(3,j),2*M_PI);
	x[i].XsigmaPoints_pred(4,j)=fmod(x[i].XsigmaPoints_pred(4,j),2*M_PI);
	x[i].XsigmaPoints_pred(5,j)=fmod(x[i].XsigmaPoints_pred(5,j),2*M_PI);

	if(params.use_ideal_meas_eqn)
	{
	    x[i].YsigmaPoints_pred.setCol(j,computeYIdeal(i,j));
	}
	else
	{
	    x[i].YsigmaPoints_pred.setCol(j,computeY(i,j));
	}
	    
    }

    x[i].x_pred=x[i].XsigmaPoints_pred*x[i].WsigmaPoints_average;
    x[i].x_pred[3]=fmod(x[i].x_pred[3],2*M_PI);
    x[i].x_pred[4]=fmod(x[i].x_pred[4],2*M_PI);
    x[i].x_pred[5]=fmod(x[i].x_pred[5],2*M_PI);
    
    x[i].y_pred=x[i].YsigmaPoints_pred*x[i].WsigmaPoints_average;
}

void UnscentedParticleFilter::computePpred(const int &i)
{
    for(size_t j=0; j<2*params.n+1; j++)
    {
        x[i].x_tilde.setCol(0,x[i].XsigmaPoints_pred.getCol(j)-x[i].x_pred);
	
        x[i].P_pred_aux=x[i].P_pred_aux+x[i].WsigmaPoints_covariance[j]*x[i].x_tilde*x[i].x_tilde.transposed();
	
    }
    x[i].P_pred=x[i].P_pred_aux + params.Q;
    
    for(size_t j=3; j<6; j++)
    {
        for(size_t k=3; k<6; k++)
        {
            x[i].P_pred(j,k)=fmod(x[i].P_pred(j,k),2*M_PI);
        }
    }
}

void UnscentedParticleFilter:: computeCorrectionMatrix(const int &i)
{
    for(size_t j=0; j<2*params.n+1; j++)
    {
        x[i].A.setCol(0,x[i].YsigmaPoints_pred.getCol(j)-x[i].y_pred);
	
        x[i].Pyy=x[i].Pyy+x[i].WsigmaPoints_covariance[j]*x[i].A*x[i].A.transposed();
	
        x[i].x_tilde.setCol(0,x[i].XsigmaPoints_pred.getCol(j)-x[i].x_pred);
	
        x[i].x_tilde(3,0)=fmod(x[i].x_tilde(3,0),2*M_PI);
        x[i].x_tilde(4,0)=fmod(x[i].x_tilde(4,0),2*M_PI);
        x[i].x_tilde(5,0)=fmod(x[i].x_tilde(5,0),2*M_PI);
	
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

    x[i].x_corr=x[i].x_pred+x[i].K*(meas-x[i].y_pred);
    x[i].x_corr[3]=fmod(x[i].x_corr[3],2*M_PI);
    x[i].x_corr[4]=fmod(x[i].x_corr[4],2*M_PI);
    x[i].x_corr[5]=fmod(x[i].x_corr[5],2*M_PI);
    x[i].P_hat=x[i].P_pred-x[i].K*x[i].Pyy*x[i].K.transposed();
    
    for(size_t j=3; j<6; j++)
    {
        for(size_t k=3; k<6; k++)
        {
            x[i].P_hat(j,k)=fmod(x[i].P_hat(j,k),2*M_PI);
        }
    }
}

double UnscentedParticleFilter::likelihood(const int &k, double &map_likelihood)
{
    double sum=0.0;
    double likelihood=1.0;
    map_likelihood=1.0;
    int initial_meas;
    
    ParticleUPF &particle=x[k];
    
    yarp::sig::Matrix H=homogeneousTransform(particle.x_corr);
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
	diff[i] = fmod(diff[i], 2 * M_PI);

    // eval the density
    yarp::sig::Matrix diff_m(x.size(), 1);
    diff_m.setCol(0, diff);

    yarp::sig::Matrix tmp(1,1);
    tmp = diff_m.transposed() * yarp::math::luinv(covariance) * diff_m;

    value = exp(-0.5 * tmp(0, 0));

    return value;
}

double UnscentedParticleFilter::tranProbability(const int &i,
						const int &j)
{
    // evaluate transition probability
    yarp::sig::Vector mean = x[j].x_corr_prev;
    mean.setSubvector(0, mean.subVector(0,2) + last_input);

    return multivariateGaussian(x[i].x_corr, mean, params.Q_prev);
}

void UnscentedParticleFilter::computeWeights(const int &i, double& sum)
{
    double standard_likelihood;
    double map_likelihood;
    double tran_prob;

    // evaluate standard likelihood and
    // likelihood corrected for MAP estimate
    standard_likelihood = likelihood(i,map_likelihood);

    // evaluate transition probability
    tran_prob = tranProbability(i, i);

    // evaluate standard weights
    x[i].weights=x[i].prev_weights * standard_likelihood * tran_prob;

    // evaluate weights corrected for MAP estimate
    x[i].weights_map=x[i].prev_weights * map_likelihood * tran_prob;
    
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
    
    x=new_x;
}

void UnscentedParticleFilter::selectionStep(const double &sum_squared)
{
    double Neff=1.0/sum_squared;
    
    if (params.resample_in_first_iters || t >= 3)
    {	
	if(Neff < params.N/20.0);
	    resampling();
    }

    if (!params.resample_in_first_iters && t < 3)
    {
	for(size_t j=0;  j<x.size(); j++)
	{
	    x[j].weights=1.0/params.N;
	}
    }

    for(size_t i=0; i<x.size(); i++)
    {
        x[i].P_corr=x[i].P_hat;
    }
}

bool UnscentedParticleFilter::configure(yarp::os::ResourceFinder &rf)
{
    yInfo()<<"UPF configuration";
    
    // assign the resource finder
    this->rf=&rf;

    // read the number of particles
    params.N=rf.find("N").asInt();
    if (rf.find("N").isNull())
	params.N=rf.check("N",yarp::os::Value(600)).asInt();
    yInfo()<<"Number of particles:"<<params.N;

    // read the center of the initial research region
    params.center0.resize(3,0.0);
    bool check=readCenter("center0",params.center0);
    if(!check)
    {
        params.center0[0]=rf.check("center0",yarp::os::Value(0.2)).asDouble();
        params.center0[1]=rf.check("center0",yarp::os::Value(0.2)).asDouble();
        params.center0[2]=rf.check("center0",yarp::os::Value(0.2)).asDouble();
    }
    yInfo()<<"Center of initial research region:"<<params.center0.toString();

    // read the radius of the initial research region
    params.radius0.resize(3,0.0);
    check=readRadius("radius0",params.radius0);
    if(!check)
    {
        params.radius0[0]=rf.check("radius0",yarp::os::Value(0.2)).asDouble();
        params.radius0[1]=rf.check("radius0",yarp::os::Value(0.2)).asDouble();
        params.radius0[2]=rf.check("radius0",yarp::os::Value(0.2)).asDouble();
    }
    yInfo()<<"Radius of initial research region:"<<params.radius0.toString();

    // read the number of DoFs
    params.n=rf.find("n").asInt();
    if (rf.find("n").isNull())
        params.n=rf.check("n",yarp::os::Value(6)).asInt();
    yInfo()<<"Number of DoF:"<<params.n;

    // read the parameter beta for the Unscented Transform
    params.beta=rf.find("beta").asDouble();
    if (rf.find("beta").isNull())
        params.beta=rf.check("beta",yarp::os::Value(35.0)).asDouble();
    yInfo()<<"Unscented Transform Beta:"<<params.beta;

    // read the parameter alpha for the Unscented Transform
    params.alpha=rf.find("alpha").asDouble();
    if (rf.find("alpha").isNull())
        params.alpha=rf.check("alpha",yarp::os::Value(1.0)).asDouble();
    yInfo()<<"Unscented Transform Alpha:"<<params.alpha;

    // read the parameter kappa for the Unscented Transform
    params.kappa=rf.find("kappa").asDouble();
    if (rf.find("kappa").isNull())
        params.kappa=rf.check("kappa",yarp::os::Value(2.0)).asDouble();
    yInfo()<<"Unscented Transform Kappa:"<<params.kappa;

    // compute the parameter lambda for the Unscented Transform
    params.lambda=pow(params.alpha,2.0)*(6+params.kappa)-6;
    yInfo()<<"Unscented Transform Lambda:"<<params.lambda;

    // read the flag resampleInFirstIters
    params.resample_in_first_iters=rf.find("resampleInFirstIters").asBool();
    if (rf.find("resampleInFirstIters").isNull())
        params.resample_in_first_iters=rf.check("resampleInFirstIters",yarp::os::Value(true)).asBool();
    yInfo()<<"UPF resample in first iterations:"<<params.resample_in_first_iters;
    
    // read the ideal measurement equation enabler
    params.use_ideal_meas_eqn=rf.find("useIdealMeasEqn").asBool();
    if (rf.find("useIdealMeasEqn").isNull())
        params.use_ideal_meas_eqn=rf.check("useIdealMeasEqn",yarp::os::Value(false)).asBool();
    yInfo()<<"UPF use ideal measurement equation :"<<params.use_ideal_meas_eqn;
    
    // read the values of the system noise covariance matrix
    yarp::sig::Vector diagQ;
    diagQ.resize(params.n,1);
    check=readDiagonalMatrix("Q",diagQ,params.n);
    if(!check)
    {
        diagQ[0]=rf.check("Q1",yarp::os::Value(0.0001)).asDouble();
        diagQ[1]=rf.check("Q2",yarp::os::Value(0.0001)).asDouble();
        diagQ[2]=rf.check("Q3",yarp::os::Value(0.0001)).asDouble();
        diagQ[3]=rf.check("Q4",yarp::os::Value(0.001)).asDouble();
        diagQ[4]=rf.check("Q5",yarp::os::Value(0.001)).asDouble();
        diagQ[5]=rf.check("Q6",yarp::os::Value(0.001)).asDouble();
    }
    yarp::sig::Matrix Q;
    params.Q.resize(params.n,params.n);
    params.Q.diagonal(diagQ);
    params.Q_prev = params.Q;
    yInfo()<<"UKF Q:"<<params.Q.toString();

    // read the noise scalar variance R
    params.R=rf.find("R").asDouble();
    if (rf.find("R").isNull())
        params.R=rf.check("R",yarp::os::Value(0.0001)).asDouble();
    yInfo()<<"UKF R (scalar):"<<params.R;
    
    // read the values of the initial state covariance matrix
    yarp::sig::Vector diagP0;
    diagP0.resize(params.n,1);
    check=readDiagonalMatrix("P0",diagP0,params.n);
    if(!check)
    {
        diagP0[0]=rf.check("P01",yarp::os::Value(0.04)).asDouble();
        diagP0[1]=rf.check("P02",yarp::os::Value(0.04)).asDouble();
        diagP0[2]=rf.check("P03",yarp::os::Value(0.04)).asDouble();
        diagP0[3]=rf.check("P04",yarp::os::Value(pow(M_PI,2.0))).asDouble();
        diagP0[4]=rf.check("P05",yarp::os::Value(pow(M_PI/2.0,2.0))).asDouble();
        diagP0[5]=rf.check("P06",yarp::os::Value(pow(M_PI,2.0))).asDouble();
    }
    params.P0.resize(params.n,params.n);
    params.P0.diagonal(diagP0);
    yInfo()<<"UKF P0:"<<params.P0.toString();

    // read triangular mesh model filename
    if (!rf.check("modelFile"))
    {
        yError()<<"model file not provided!";
        return false;
    }
    std::string modelFileName=rf.find("modelFile").asString().c_str();
    // read the polyhedron from a .OFF file
    std::ifstream modelFile(modelFileName.c_str());
    if (!modelFile.is_open())
    {
        yError()<<"problem opening model file!";
        return false;
    }
    modelFile>>getModel();
    if (modelFile.bad())
    {
        yError()<<"problem reading model file!";
        modelFile.close();
        return false;
    }
    modelFile.close();
    yInfo()<<"Model file loaded successfully";

    // init CGAL
    GeometryCGAL::init();

    return true;
}

void UnscentedParticleFilter::init()
{
    // init index of iteration
    t=0;

    // clear system input
    last_input.resize(3, 0.0);
    input.resize(3, 0.0);

    // initialize particles and sample from initial search region
    x.assign(params.N,ParticleUPF());
    initialRandomize();

    // complete UPF initialization
    initializationUPF();
}

void UnscentedParticleFilter::setNewInput(const yarp::sig::Vector &in)
{
    // set the new system input received
    input = in;
}

void UnscentedParticleFilter::setNewMeasure(const Measure& m)
{
    // set the new measure
    curr_meas = m;
}

void UnscentedParticleFilter::setRealPose(const yarp::sig::Vector &pose)
{
    // assign the current real pose
    real_pose = pose;
}

void UnscentedParticleFilter::setQ(const yarp::sig::Vector &covariance)
{
    // assign a new covariance matrix
    params.Q.diagonal(covariance);
}

void UnscentedParticleFilter::step()
{   
    t++;

    double sum=0.0;
    double sum_squared=0.0;

    // normal distribution generator
    yarp::math::RandnScalar normal_gen;
    normal_gen.init();

    // process all the particles
    for(size_t i=0; i<x.size(); i++ )
    {
        resizeParticle(i);
        initializeUKFMatrix(i);
        computeSigmaPoints(i);
     	predictionStep(i, normal_gen);
        computePpred(i);
        computeCorrectionMatrix(i);
        x[i].K=x[i].Pxy*luinv(x[i].Pyy);
        correctionStep(i);
        computeWeights(i, sum);
    }

    // normalize weights
    for (size_t i=0; i<x.size(); i++)
    {
        normalizeWeights(i, sum, sum_squared);
    }

    // resampling strategies
    selectionStep(sum_squared);

    // update system input
    last_input = input;

    // update previous value of x_corr
    for (size_t i=0; i<x.size(); i++)
    {
	x[i].x_corr_prev = x[i].x_corr;
    }
    // update previous value of Q
    params.Q_prev = params.Q;
}

yarp::sig::Vector UnscentedParticleFilter::getEstimate()
{
    // corrected weights are not normalized at each step since not required
    // normalization is done here
    double sum_weights=0.0;
    for(size_t i=0; i<x.size(); i++)
    {
	sum_weights+=x[i].weights_map;
    }
    for(size_t i=0; i<x.size(); i++)
    {
	x[i].weights_map/=sum_weights;
    }

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
	meas_likelihood = likelihood(i, tmp);

	// store probability
        probability_per_particle.push_back(meas_likelihood * sum_tran_probability);
    }

    // update the weights for the next step
    for(size_t i=0; i<x.size(); i++)
    {
	x[i].prev_weights = x[i].weights;
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

    return x[i_max_prob].x_corr;
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

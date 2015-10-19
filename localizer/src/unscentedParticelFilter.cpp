

/*
 * Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Ugo Pattacini
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#include <iostream>
#include <string>
#include <fstream>
#include <algorithm>
#include <cmath>


#include <yarp/os/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/api.h>
#include <yarp/math/Rand.h>
#include <yarp/math/RandnVector.h>
#include <yarp/math/RandnScalar.h>
#include <yarp/math/SVD.h>
#include <yarp/os/ResourceFinder.h>




#include <iCub/ctrl/math.h>
#include <CGAL/IO/Polyhedron_iostream.h>

#include "headers/unscentedParticleFilter.h"



typedef CGAL::Aff_transformation_3<K> Affine;

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;



/*******************************************************************************/
UnscentedParticleFilter::UnscentedParticleFilter() : GeometryCGAL()
{
    
    parameters=new ParametersUPF;   // owned by GeometryCGAL
}


/*******************************************************************************/
ParametersUPF &UnscentedParticleFilter::get_parameters()
{
    return *static_cast<ParametersUPF*>(parameters);
}


/*******************************************************************************/
void UnscentedParticleFilter::initialRandomize()
{
    for (size_t i=0; i<x.size(); i++)
    {
        ParticleUPF &particle=x[i];
        ParametersUPF &params=get_parameters();
        
        particle.x_corr[0]=Rand::scalar(params.center0[0]-params.radius0[0],params.center0[0]+params.radius0[0]);
        particle.x_corr[1]=Rand::scalar(params.center0[1]-params.radius0[1],params.center0[1]+params.radius0[1]);
        particle.x_corr[2]=Rand::scalar(params.center0[2]-params.radius0[2],params.center0[2]+params.radius0[2]);
        particle.x_corr[3]=Rand::scalar(0,2*M_PI);
        particle.x_corr[4]=Rand::scalar(0,M_PI);
        particle.x_corr[5]=Rand::scalar(0,2*M_PI);
        
   
    }
}


/*******************************************************************************/
void UnscentedParticleFilter::init()
{   

    t0=Time::now();
    GeometryCGAL::init();
    ParametersUPF &params=get_parameters();
    
    t=0;
    n=params.n;
    p=params.p;

    x.assign(params.N,ParticleUPF());
	initialRandomize();

	initializationUPF();
	
	
}

/*******************************************************************************/
bool UnscentedParticleFilter::step()
{   

    t++;
    cout<<"t "<<t<<"\n";
    
    cout<<"modified algorithm "<<endl;
    
    ParametersUPF &params=get_parameters();
    cout<<"num tot meas "<<params.numMeas<<endl;
    
	if( t>params.numMeas)
	{	
		return true;
	}
	
    
	double sum=0.0;
	double sum_squared=0.0;
	double Neff=0.0;

	
	yarp::sig::Vector random;
	random.resize(n,0.0);
	
	yarp::sig::Matrix cholQ;
	cholQ(n,n);

	RandnScalar prova;


	//initialize Matrix for UKF
	for(size_t i=0; i<x.size(); i++ )
	{
		initializeUKFMatrix(i);

	}

	//compute sigmaPoints
	for(size_t i=0; i<x.size(); i++ )
	{
		computeSigmaPoints(i);
		
	}

	//propagate particle in the future
	for(size_t i=0; i<x.size(); i++ )
	{
	
		for(size_t j=0; j<2*n+1; j++)
		{
		
			for( size_t l=0; l<n; l++)
			{
					
			random[l]=prova.RandnScalar::get(0.0, 1.0);
		
			}
	
		cholQ=params.Q;
	
		//since Q is diagonal, chol(Q) is sqrt of its member on diagonal
			
		for(size_t k=0; k<6; k++)
		{
			for( size_t l=0; l<n; l++)
			{
				cholQ(k,l)=sqrt(cholQ(k,l));
		
			}
		}


		x[i].XsigmaPoints_pred.setCol(j,x[i].XsigmaPoints_corr.getCol(j)+cholQ*random);

		x[i].XsigmaPoints_pred(3,j)=fmod(x[i].XsigmaPoints_pred(3,j),2*M_PI);
		x[i].XsigmaPoints_pred(4,j)=fmod(x[i].XsigmaPoints_pred(4,j),2*M_PI);
		x[i].XsigmaPoints_pred(5,j)=fmod(x[i].XsigmaPoints_pred(5,j),2*M_PI);

		x[i].YsigmaPoints_pred.setCol(j,compute_y(t,i,j));
	
		}
		
	}
	 
	for(size_t i=0; i<x.size(); i++ )
	{
	 	predictionStep(i);
	}
;
	for(size_t i=0; i<x.size(); i++ )
	{
		
		computePpred(i);
	}


	//correction
	for(size_t i=0; i<x.size(); i++ )
	{
		computeCorrectionMatrix(i);
	}

	for(size_t i=0; i<x.size(); i++)
	{
	
		x[i].K=x[i].Pxy*luinv(x[i].Pyy);
	}

	for(size_t i=0; i<x.size(); i++)
	{
		correctionStep(i);
	}

	//computer weights
	for(size_t i=0; i<x.size(); i++ )
	{
		computeWeights(i, sum);

	}

	for (size_t i=0; i<x.size(); i++)
	{
		normalizeWeights(i, sum, sum_squared);

	}
	
	findMostSignificantParticle();

	selectionStep(Neff,sum_squared);

    
    return false;
    
}


/*******************************************************************************/
Vector UnscentedParticleFilter::finalize()
{  
	//cout<<"we are in finalize"<<endl;

	Vector error_indices;
	error_indices.resize(3,0.0);
	
	result.resize(10,0.0);
	

   
    
    ms_particle1.pos=x_most.back();
    performanceIndex(ms_particle1);
   
   
    
    
  	result[0]=ms_particle1.pos[0];
  	result[1]=ms_particle1.pos[1];
  	result[2]=ms_particle1.pos[2];
  	result[3]=ms_particle1.pos[3];
  	result[4]=ms_particle1.pos[4];
  	result[5]=ms_particle1.pos[5];
  	result[6]=ms_particle1.error_index;
    dt=Time::now()-t0;
    result[7]=dt;
    
    cout<<"RESULT WITH HIGHEST WEIGHT "<<result.toString().c_str()<<endl;
    
    
    //let's try to use instead of particle with highest weight
    // particle with highest density
    Vector result2;
    result2=particleDensity();
    
    cout<<"RESULT WITH HIGHEST DENSITY "<<result2.toString().c_str()<<endl;
    
    
     dt=Time::now()-t0;
    result[8]=dt;
    
    //save all the result wwith highest density in ms_particle2
    ms_particle2.pos=result2;
   
    
    
   
                   
    performanceIndex(ms_particle2);
    cout<<"error_index"<<ms_particle2.error_index<<endl;
    
    
    
    
     Vector result3;
    result3=particleDensity2();
    
     dt=Time::now()-t0;
    result[9]=dt;
    
    cout<<"RESULT WITH HIGHEST DENSITY 2"<<result3.toString().c_str()<<endl;
    
    //save all the result wwith highest density in ms_particle2
    
    ms_particle3.pos=result3;
   
    
   
   
                   
    performanceIndex(ms_particle3);
    cout<<"error_index"<<ms_particle3.error_index<<endl;
    
    
     Matrix H=rpr(ms_particle1.pos.subVector(3,5));
    Affine affine(H(0,0),H(0,1),H(0,2),ms_particle1.pos[0],
                  H(1,0),H(1,1),H(1,2),ms_particle1.pos[1],
                  H(2,0),H(2,1),H(2,2),ms_particle1.pos[2]);
                           
    std::transform(model.points_begin(),model.points_end(),
                   model.points_begin(),affine);
                   
                   
    error_indices[0]=ms_particle1.error_index;
    error_indices[1]=ms_particle2.error_index;
    error_indices[2]=ms_particle3.error_index;
	
  
    return error_indices;
}
 
/*******************************************************************************/
Vector UnscentedParticleFilter::particleDensity()
{	
	
	
	
	ParametersUPF &params=get_parameters();

	
	yarp::sig::Vector random;
	random.resize(n,0.0);
	
	yarp::sig::Matrix cholQ;
	cholQ(n,n);

	RandnScalar prova;


	//initialize Matrix for UKF
	for(size_t i=0; i<x.size(); i++ )
	{
		initializeUKFMatrix(i);

	}

	//compute sigmaPoints
	for(size_t i=0; i<x.size(); i++ )
	{
		computeSigmaPoints(i);
		
	}

	//propagate particle in the future
	for(size_t i=0; i<x.size(); i++ )
	{
	
		for(size_t j=0; j<2*n+1; j++)
		{
		
			for( size_t l=0; l<n; l++)
			{
					
			random[l]=prova.RandnScalar::get(0.0, 1.0);
		
			}
	
		cholQ=params.Q;
	
		//since Q is diagonal, chol(Q) is sqrt of its member on diagonal
			
		for(size_t k=0; k<6; k++)
		{
			for( size_t l=0; l<n; l++)
			{
				cholQ(k,l)=sqrt(cholQ(k,l));
		
			}
		}


		x[i].XsigmaPoints_pred.setCol(j,x[i].XsigmaPoints_corr.getCol(j)+cholQ*random);

		x[i].XsigmaPoints_pred(3,j)=fmod(x[i].XsigmaPoints_pred(3,j),2*M_PI);
		x[i].XsigmaPoints_pred(4,j)=fmod(x[i].XsigmaPoints_pred(4,j),2*M_PI);
		x[i].XsigmaPoints_pred(5,j)=fmod(x[i].XsigmaPoints_pred(5,j),2*M_PI);

		x[i].YsigmaPoints_pred.setCol(j,compute_y(t,i,j));
	
		}
		
	}
	 
	for(size_t i=0; i<x.size(); i++ )
	{
	 	predictionStep(i);
	}
	
	
	
	
	
	
	
	

	
	
	
	deque<double> Mahalanobis_distance;

	
	
	for(size_t i=0; i<x.size(); i++)
	{
		Mahalanobis_distance.push_back(0);
		for(size_t j=0; j<x.size(); j++)
		{
			Matrix diff(6,1);
		
			diff(0,0)=x[i].x_pred(0)-x[j].x_pred(0);
			diff(1,0)=x[i].x_pred(1)-x[j].x_pred(1);
			diff(2,0)=x[i].x_pred(2)-x[j].x_pred(2);
			diff(3,0)=fmod(x[i].x_pred(3)-x[j].x_pred(3),2*M_PI);
			diff(4,0)=fmod(x[i].x_pred(4)-x[j].x_pred(4),2*M_PI);
			diff(5,0)=fmod(x[i].x_pred(5)-x[j].x_pred(5),2*M_PI);
			
		
			Matrix temp(1,1);
			temp=diff.transposed()*luinv(params.Q)*diff;
			Mahalanobis_distance[i]=Mahalanobis_distance[i]+sqrt(temp(0,0));
			
		}
		
	}
	
	double Mahalanobis_minimum_distance;
	int i_min_dist;
	Mahalanobis_minimum_distance=10000000000;
	for(size_t i=0; i<x.size(); i++)
	{
		if(	Mahalanobis_minimum_distance> Mahalanobis_distance[i])
		{
			
			Mahalanobis_minimum_distance=Mahalanobis_distance[i];
			i_min_dist=i;
			
		}
		
		
	}
	
	
	cout<<"i_min_dist "<<i_min_dist<<endl;
	return x[i_min_dist].x_pred;
	

	
	
	
	

}
/*******************************************************************************/

double UnscentedParticleFilter::likelihood(const int &t, const int &k)
{
	double sum=0.0;
    ParametersUPF &params=get_parameters();
	double likelihood=1.0;
	int initial_meas;

	ParticleUPF &particle=x[k];

	Matrix H=rpr(particle.x_corr.subVector(3,5));
	H.transposed();
	
	H(0,3)=particle.x_corr[0];
	H(1,3)=particle.x_corr[1];
	H(2,3)=particle.x_corr[2];
		
	H=SE3inv(H);

	if(params.window_width==params.N)
	initial_meas=0;
	else
	{
		if(t-params.window_width<0)
			initial_meas=0;
		else
			initial_meas=t-params.window_width;
	}
		
	//	cout<<"INITIAL MEAS "<<initial_meas<<endl;
		
//	if (t<=params.window_width)
//	{
//		for (size_t i=initial_meas; i<t; i++)
//		{
			
//			Point &m=measurements[i];
			
//				
//			double x=H(0,0)*m[0]+H(0,1)*m[1]+H(0,2)*m[2]+H(0,3);
//			double y=H(1,0)*m[0]+H(1,1)*m[1]+H(1,2)*m[2]+H(1,3);
//			double z=H(2,0)*m[0]+H(2,1)*m[1]+H(2,2)*m[2]+H(2,3);
			
			
			//modification for not double counting
				
//			if (i==t-1)
//				likelihood=likelihood*exp(-0.5*tree.squared_distance(Point(x,y,z))/(params.R(0,0))*t);
//			else
//				likelihood=likelihood*exp(-0.5*tree.squared_distance(Point(x,y,z))/(params.R(0,0)));
				
//		}
//	}
//	else
//	{
		int count;
		count=0;
			for (size_t i=initial_meas; i<t; i++)
			{
		
				Point &m=measurements[i];
			
				
				double x=H(0,0)*m[0]+H(0,1)*m[1]+H(0,2)*m[2]+H(0,3);
				double y=H(1,0)*m[0]+H(1,1)*m[1]+H(1,2)*m[2]+H(1,3);
				double z=H(2,0)*m[0]+H(2,1)*m[1]+H(2,2)*m[2]+H(2,3);
				
				
				//modification for not double counting
					
				//if (i==t-1)
				//	likelihood=likelihood*exp(-0.5*tree.squared_distance(Point(x,y,z))/(params.R(0,0))*t);
				//if(i==initial_meas-1)
				//	likelihood=likelihood*exp(0.5*tree.squared_distance(Point(x,y,z))/(params.R(0,0))*(t-1));
				//else
				count++;
				if(t==params.numMeas)
				{
					
					
						likelihood=likelihood*exp(-0.5*tree.squared_distance(Point(x,y,z))/(params.R(0,0))*(count));
				
					
						
				}
			
				else
				{
					likelihood=likelihood*exp(-0.5*tree.squared_distance(Point(x,y,z))/(params.R(0,0)));
				}
				
			}
		
	//}


   return likelihood;

}	



/*******************************************************************************/


Vector UnscentedParticleFilter::particleDensity2()
{	
	
	
	
	ParametersUPF &params=get_parameters();

	

	
	
	//compute number of particles in each neighrboard
	deque<int> number_per_neigh;
	deque<Vector> particle_per_particle;
	deque<double> Mahalanobis_distance;
	
	Vector neigh=params.neigh;
	double percentage=params.percent;
	
	
	for(size_t i=0; i<x.size(); i++)
	{
		cout<<"x i pred "<<x[i].x_pred.toString().c_str()<<endl;
		Vector index_of_particle;
		
		Vector check_on_dimensions;
		check_on_dimensions.resize(6,0.0);
		int count=0;
		
		for(size_t j=0; j<x.size(); j++)
		{
			cout<<"x j pred "<<x[j].x_pred.toString().c_str()<<endl;
        	int sum=0;
			if((x[j].x_pred[0]<=x[i].x_pred[0]+neigh[0]) && (x[j].x_pred[0]>=x[i].x_pred[0]-neigh[0]))
			{
				check_on_dimensions[0]=1;

			}
				if((x[j].x_pred[1]<=x[i].x_pred[1]+neigh[1]) && (x[j].x_pred[1]>=x[i].x_pred[1]-neigh[1]))
			{
				check_on_dimensions[1]=1;

			}
				if((x[j].x_pred[2]<=x[i].x_pred[2]+neigh[2]) && (x[j].x_pred[2]>=x[i].x_pred[2]-neigh[2]))
			{
				check_on_dimensions[2]=1;

			}
		
				if((x[j].x_pred[4]<=fmod(x[i].x_pred[4]+neigh[4],2*M_PI)) && (x[j].x_pred[4]>=fmod(x[i].x_pred[4]-neigh[4],2*M_PI)))
			{
				check_on_dimensions[4]=1;

			}
				if((x[j].x_pred[5]<=fmod(x[i].x_pred[5]+neigh[5],2*M_PI)) && (x[j].x_pred[5]>=fmod(x[i].x_pred[5]-neigh[5],2*M_PI)))
			{
				check_on_dimensions[5]=1;

			}
				if((x[j].x_pred[3]<=fmod(x[i].x_pred[3]+neigh[3],2*M_PI)) && (x[j].x_pred[3]>=fmod(x[i].x_pred[3]-neigh[3],2*M_PI)))
			{
				check_on_dimensions[3]=1;

			}
			
         
			sum=check_on_dimensions[0]+check_on_dimensions[1]+check_on_dimensions[2]+check_on_dimensions[3]+check_on_dimensions[4]+check_on_dimensions[5];
            
         	
			if(sum==6)
			{
				count++;
				index_of_particle.push_back(j);
				cout<<"count "<<count<<endl;
				cout<<"particle included "<<index_of_particle.toString().c_str()<<endl;
				
				
				
			}

		}
		
	
		particle_per_particle.push_back(index_of_particle);
		number_per_neigh.push_back(count);
	
	}

	
	int max_numer_per_part;
	max_numer_per_part=0;
	
	for(size_t i=0; i<number_per_neigh.size();i++)
	{
		if(max_numer_per_part<number_per_neigh[i])
		{
			max_numer_per_part=number_per_neigh[i];
			cout<<"max num per part "<<max_numer_per_part<<endl;
			
		}
			
	}
	
	cout<<"max num per part "<<max_numer_per_part<<endl;

	int l;
	
	for(size_t i=0; i<x.size(); i++)
	{
		Mahalanobis_distance.push_back(0);
		if(number_per_neigh[i]>=percentage*max_numer_per_part)
		{
		
			for(size_t j=0; j<particle_per_particle[i].size(); j++)
			{
					Vector temp2;
					temp2=particle_per_particle[i];
					l=temp2[j];
				
					Matrix diff(6,1);
				
					diff(0,0)=x[i].x_pred(0)-x[l].x_pred(0);
					diff(1,0)=x[i].x_pred(1)-x[l].x_pred(1);
					diff(2,0)=x[i].x_pred(2)-x[l].x_pred(2);
					diff(3,0)=fmod(x[i].x_pred(3)-x[l].x_pred(3),2*M_PI);
					diff(4,0)=fmod(x[i].x_pred(4)-x[l].x_pred(4),2*M_PI);
					diff(5,0)=fmod(x[i].x_pred(5)-x[j].x_pred(5),2*M_PI);
					
				
					Matrix temp(1,1);
					temp=diff.transposed()*luinv(params.Q)*diff;
					Mahalanobis_distance[i]=Mahalanobis_distance[i]+sqrt(temp(0,0));
						
			}
		}
		
	}
	
	


	
	double Mahalanobis_minimum_distance;
	int i_min_dist;
	Mahalanobis_minimum_distance=10000000000;
	
	for(size_t i=0; i<Mahalanobis_distance.size(); i++)
	{
		if(number_per_neigh[i]>=percentage*max_numer_per_part)
		{
			if(	Mahalanobis_minimum_distance> Mahalanobis_distance[i])
			{
				
				Mahalanobis_minimum_distance=Mahalanobis_distance[i];
				i_min_dist=i;
				
			}
		}
		
		
	}
	
	
	cout<<"i_min_dist "<<i_min_dist<<endl;
	return x[i_min_dist].x_pred;
	

	
	
	
	

}
/*******************************************************************************/



/*******************************************************************************/
void UnscentedParticleFilter::resampling()
{	
	std::deque<ParticleUPF> new_x;
	Vector c,u, new_index;
	ParametersUPF &params=get_parameters();

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
	u[0]=Rand::scalar(0,1)/params.N;

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

/*******************************************************************************/
void UnscentedParticleFilter::performanceIndex( MsParticleUPF &ms_particle)
{
  
	ms_particle.error_index=0;

	Matrix H=rpr(ms_particle.pos.subVector(3,5));
	H.transposed();
	H(0,3)=ms_particle.pos[0];
	H(1,3)=ms_particle.pos[1];
	H(2,3)=ms_particle.pos[2];
	H=SE3inv(H);

    
		
	for (size_t i=0; i<measurements.size(); i++)
	{
		Point &m=measurements[i];
		

		double x=H(0,0)*m[0]+H(0,1)*m[1]+H(0,2)*m[2]+H(0,3);
		double y=H(1,0)*m[0]+H(1,1)*m[1]+H(1,2)*m[2]+H(1,3);
		double z=H(2,0)*m[0]+H(2,1)*m[1]+H(2,2)*m[2]+H(2,3);
		


		ms_particle.error_index+=sqrt(tree.squared_distance(Point(x,y,z)));
		
	}
	
  	ms_particle.error_index=ms_particle.error_index/measurements.size();


}

/*******************************************************************************/

Matrix UnscentedParticleFilter::rpr(const Vector &particle)
{
	

	Matrix H(4,4);
	double phi=particle[0];
	double theta=particle[1];
	double psi=particle[2];
    H(0,0)=cos(phi)*cos(theta)*cos(psi)-sin(phi)*sin(psi);
    H(0,1)=-cos(phi)*cos(theta)*sin(psi)-sin(phi)*cos(psi);
    H(0,2)=cos(phi)*sin(theta);
    H(1,0)=sin(phi)*cos(theta)*cos(psi)+cos(phi)*sin(psi);
    H(1,1)=-sin(phi)*cos(theta)*sin(psi)+cos(phi)*cos(psi);
    H(1,2)=sin(phi)*sin(theta) ;
    H(2,0)= -sin(theta)*cos(psi);
    H(2,1)=sin(theta)*sin(psi) ;
    H(2,2)=cos(theta) ;
   
    
    return H;
    
}

/*******************************************************************************/
void UnscentedParticleFilter::computeSigmaPoints(const int &i)
{
	ParametersUPF &params=get_parameters();
	yarp::sig::Vector aux;
	
	
	yarp::sig::Matrix S,U,V,G;
	yarp::sig::Vector s;
	s.resize(n,0.0);
	U.resize(n,n);
	S.resize(n,n);
	V.resize(n,n);
	G.resize(n,n);

	 
	
	SVD((n+params.lambda)*x[i].P_corr,U,s,V);
		
	
	for(size_t k=0; k<n; k++)
	{
			s[k]=sqrt(s[k]);
		
		
	}
	S.diagonal(s);


	G=U*S;

	x[i].XsigmaPoints_corr.setCol(0,x[i].x_corr);
		
	for(size_t j=1; j<n+1; j++)
	{
	
		x[i].XsigmaPoints_corr.setCol(j,x[i].x_corr+G.getCol(j-1));
	
		x[i].XsigmaPoints_corr(3,j)=fmod(x[i].XsigmaPoints_corr(3,j),2*M_PI);
		x[i].XsigmaPoints_corr(4,j)=fmod(x[i].XsigmaPoints_corr(4,j),2*M_PI);
		x[i].XsigmaPoints_corr(5,j)=fmod(x[i].XsigmaPoints_corr(5,j),2*M_PI);
	}
	
	for(size_t j=n+1; j<2*n+1; j++)
	{
		
		x[i].XsigmaPoints_corr.setCol(j,x[i].x_corr-G.getCol(j-1-n));
		
		x[i].XsigmaPoints_corr(3,j)=fmod(x[i].XsigmaPoints_corr(3,j),2*M_PI);
		x[i].XsigmaPoints_corr(4,j)=fmod(x[i].XsigmaPoints_corr(4,j),2*M_PI);
		x[i].XsigmaPoints_corr(5,j)=fmod(x[i].XsigmaPoints_corr(5,j),2*M_PI);
	}
	
	
	x[i].WsigmaPoints_covariance[0]=params.lambda/(n+params.lambda)+1-pow(params.alpha,2.0)+params.beta;
	x[i].WsigmaPoints_average[0]=params.lambda/(n+params.lambda);
	
	
	for(size_t j=1; j<2*n+1; j++)
	{
		x[i].WsigmaPoints_covariance[j]=1/(2*(n+params.lambda));
		x[i].WsigmaPoints_average[j]=1/(2*(n+params.lambda));
		
	}
	
	
}

/*******************************************************************************/
yarp::sig::Vector UnscentedParticleFilter::compute_y(const int &t, const int &k, const int &j)
{

    ParametersUPF &params=get_parameters();
	Point y_pred;
	yarp::sig::Vector out;
	out.resize(p,0.0);

	ParticleUPF &particle=x[k];

	Matrix Hm=rpr(particle.XsigmaPoints_pred.getCol(j).subVector(3,5));
	
	Hm.transposed();
	
	Hm(0,3)=x[k].XsigmaPoints_pred(0,j);
	Hm(1,3)=x[k].XsigmaPoints_pred(1,j);
	Hm(2,3)=x[k].XsigmaPoints_pred(2,j);

		
	Hm=SE3inv(Hm);
	Point &m=measurements[t-1];
	double x=Hm(0,0)*m[0]+Hm(0,1)*m[1]+Hm(0,2)*m[2]+Hm(0,3);
	double y=Hm(1,0)*m[0]+Hm(1,1)*m[1]+Hm(1,2)*m[2]+Hm(1,3);
	double z=Hm(2,0)*m[0]+Hm(2,1)*m[1]+Hm(2,2)*m[2]+Hm(2,3);

	y_pred=tree.closest_point(Point(x,y,z));


	Hm=SE3inv(Hm);
			
	out[0]=Hm(0,0)*y_pred[0]+Hm(0,1)*y_pred[1]+Hm(0,2)*y_pred[2]+Hm(0,3);
	out[1]=Hm(1,0)*y_pred[0]+Hm(1,1)*y_pred[1]+Hm(1,2)*y_pred[2]+Hm(1,3);
	out[2]=Hm(2,0)*y_pred[0]+Hm(2,1)*y_pred[1]+Hm(2,2)*y_pred[2]+Hm(2,3);
			


    return out;

}
/*******************************************************************************/
void UnscentedParticleFilter::predictionStep(const int &i)
{
	x[i].x_pred=x[i].XsigmaPoints_pred*x[i].WsigmaPoints_average;
	x[i].x_pred[3]=fmod(x[i].x_pred[3],2*M_PI);
	x[i].x_pred[4]=fmod(x[i].x_pred[4],2*M_PI);
	x[i].x_pred[5]=fmod(x[i].x_pred[5],2*M_PI);

	x[i].y_pred=x[i].YsigmaPoints_pred*x[i].WsigmaPoints_average;

}
/*******************************************************************************/
void UnscentedParticleFilter::initializeUKFMatrix(const int &i)
{
	x[i].Pyy.zero();
	x[i].Pxy.zero();
	x[i].K.zero();
	x[i].P_pred_aux.zero();
}
/*******************************************************************************/
void UnscentedParticleFilter::computePpred(const int &i)
{
	for(size_t j=0; j<2*n+1; j++)
	{
		x[i].x_tilde.setCol(0,x[i].XsigmaPoints_pred.getCol(j)-x[i].x_pred);
	
		x[i].P_pred_aux=x[i].P_pred_aux+x[i].WsigmaPoints_covariance[j]*x[i].x_tilde*x[i].x_tilde.transposed();

				
	}
	x[i].P_pred=x[i].P_pred_aux;
		
	for(size_t j=3; j<6; j++)
	{
		for(size_t k=3; k<6; k++)
		{
			x[i].P_pred(j,k)=fmod(x[i].P_pred(j,k),2*M_PI);
		}
	}
}
/*******************************************************************************/
void UnscentedParticleFilter:: computeCorrectionMatrix(const int &i)
{
	for(size_t j=0; j<2*n+1; j++)
	{
		x[i].A.setCol(0,x[i].YsigmaPoints_pred.getCol(j)-x[i].y_pred);
	
		x[i].Pyy=x[i].Pyy+x[i].WsigmaPoints_covariance[j]*x[i].A*x[i].A.transposed();
			
		x[i].x_tilde.setCol(0,x[i].XsigmaPoints_pred.getCol(j)-x[i].x_pred);
			
		x[i].x_tilde(3,0)=fmod(x[i].x_tilde(3,0),2*M_PI);
		x[i].x_tilde(4,0)=fmod(x[i].x_tilde(4,0),2*M_PI);
		x[i].x_tilde(5,0)=fmod(x[i].x_tilde(5,0),2*M_PI);
		
		x[i].Pxy=x[i].Pxy+x[i].WsigmaPoints_covariance[j]*x[i].x_tilde*x[i].A.transposed();
		
		}
}
/*******************************************************************************/
void UnscentedParticleFilter::correctionStep(const int &i)
{
	yarp::sig::Vector meas;
    meas.resize(p,0.0);
	Point &m=measurements[t-1];
	meas[0]=m[0];
	meas[1]=m[1];
	meas[2]=m[2];
		
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
/*******************************************************************************/
void UnscentedParticleFilter::computeWeights(const int &i, double& sum)
{
	x[i].weights=x[i].weights*likelihood(t,i);

	sum+=x[i].weights;
}
/*******************************************************************************/
void UnscentedParticleFilter:: normalizeWeights(const int &i,const double &sum, double &sum_squared)
{
	x[i].weights=x[i].weights/sum;
	sum_squared+=pow(x[i].weights,2.0);
}
/*******************************************************************************/
void UnscentedParticleFilter::findMostSignificantParticle()
{
	double larger_weights=0.0;
	int index_most=0;
 
    //find most significant particle
    for(size_t i=0; i< x.size(); i++)
    { 	
   	 
   		if(larger_weights<=x[i].weights|| i==0)
  		{
   			larger_weights=x[i].weights;
   			index_most=i;
   		
   		}
    }
  
   cout<<"index_most "<<index_most<<endl;
	x_most.push_back(x[index_most].x_corr);
}
/*******************************************************************************/
void UnscentedParticleFilter::selectionStep(double &Neff,const double &sum_squared)
{
	ParametersUPF &params=get_parameters();
	
	Neff=1.0/sum_squared;
  
	if(Neff< params.N/20.0 && t>=3)
	{
		resampling();
	}
	if( t<3)
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
/*******************************************************************************/
void UnscentedParticleFilter::initializationUPF(){
	
	 ParametersUPF &params=get_parameters();
	 for(size_t i=0; i<x.size(); i++ )
	 {
		x[i].weights=1.0/params.N;
		x[i].P_corr=params.P0;
		x[i].P_hat.zero();
        x[i].P_pred.zero();
		x[i].XsigmaPoints_corr.resize(n,2*n+1);
		x[i].XsigmaPoints_pred.resize(n,2*n+1);
		x[i].YsigmaPoints_pred.resize(p,2*n+1);
		x[i].WsigmaPoints_average.resize(2*n+1,0.0);
		x[i].WsigmaPoints_covariance.resize(2*n+1,0.0);
     }

	
}

/*******************************************************************************/

void UnscentedParticleFilter::solve()
{
	bool finished=false;
	while(finished==false)
	{
		finished=UnscentedParticleFilter::step();
	}

		
}
/*******************************************************************************/
bool UnscentedParticleFilter::readMeasurements(ifstream &fin)
{
    ParametersUPF &params=get_parameters();
    int state=0;
    int nPoints;
    char line[255];
	params.numMeas=0;
        
    while (!fin.eof())
    {
        fin.getline(line,sizeof(line),'\n');
        Bottle b(line);
        Value firstItem=b.get(0);
        bool isNumber=firstItem.isInt() || firstItem.isDouble();
            
        if (state==0)
        {
            string tmp=firstItem.asString().c_str();
            std::transform(tmp.begin(),tmp.end(),tmp.begin(),::toupper);
            if (tmp=="OFF")
                state++;
            }
            else if (state==1)
            {
                if (isNumber)
                {
                    nPoints=firstItem.asInt();
                    state++;
                }
            }
            else if (state==2)
            {
                if (isNumber && (b.size()>=3))
                {
                    get_measurements().push_back(Point(b.get(0).asDouble(),
                                                       b.get(1).asDouble(),
                                                       b.get(2).asDouble()));
					params.numMeas++;
                    
                    if (--nPoints<=0)
                        return true;
                }
            }
        }
        
        return false;
}

/*******************************************************************************/    
    
bool UnscentedParticleFilter::configure(ResourceFinder &rf)
{
	 this->rf=&rf;
  
        if (!rf.check("modelFile"))
        {
            yError()<<"model file not provided!";
            return false;
        }
    
        if (!rf.check("measurementsFile"))
        {
            yError()<<"measurements file not provided!";
            return false;
        }
        
       
       
        string modelFileName=rf.find("modelFile").asString().c_str();
      
        string measurementsFileName=rf.find("measurementsFile"). asString().c_str();
        
        if(rf.find("measurementsFile").isNull())
        	string measurementsFileName="../measurements.off";
                                    
 
        ParametersUPF &parameters=get_parameters();
        parameters.N=rf.find("N").asInt();
        if (rf.find("N").isNull())
        parameters.N=rf.check("N",Value(600)).asInt();
        
      
    
		parameters.center0.resize(3,0.0);
		bool check=readCenter("center0",parameters.center0);
	
		if(!check)
		{
			parameters.center0[0]=rf.check("center0",Value(0.2)).asDouble();
			parameters.center0[1]=rf.check("center0",Value(0.2)).asDouble();
			parameters.center0[2]=rf.check("center0",Value(0.2)).asDouble();
		}
	
		parameters.radius0.resize(3,0.0);
	
		check=readRadius("radius0",parameters.center0);
	
		if(!check)
		{
			parameters.radius0[0]=rf.check("radius0",Value(0.2)).asDouble();
			parameters.radius0[1]=rf.check("radius0",Value(0.2)).asDouble();
			parameters.radius0[2]=rf.check("radius0",Value(0.2)).asDouble();
		}

        parameters.n=rf.find("n").asInt();
        if (rf.find("n").isNull())
        	parameters.n=rf.check("n",Value(6)).asInt();
        
        parameters.p=rf.find("p").asInt();
        if (rf.find("p").isNull())
        	parameters.p=rf.check("p",Value(3)).asInt();
        
        parameters.beta=rf.find("beta").asDouble();
        if (rf.find("beta").isNull())
        	parameters.beta=rf.check("beta",Value(35.0)).asDouble();
        
        
        parameters.alpha=rf.find("alpha").asDouble();
        if (rf.find("alpha").isNull())
        	parameters.alpha=rf.check("alpha",Value(1.0)).asDouble();
        
        parameters.kappa=rf.find("kappa").asDouble();
        if (rf.find("kappa").isNull())
        	parameters.kappa=rf.check("kappa",Value(2.0)).asDouble();
        
        parameters.lambda=pow(parameters.alpha,2.0)*(6+parameters.kappa)-6;
        
        
        parameters.percent=rf.find("percentage").asDouble();
        if (rf.find("percentage").isNull())
        	parameters.percent=rf.check("percentage",Value(0.85)).asDouble();
        	
        	
       
        
        yarp::sig::Vector diagQ;
        diagQ.resize(parameters.n,1);
        check=readDiagonalMatrix("Q",diagQ,parameters.n);
	
		if(!check)
		{
        	diagQ[0]=rf.check("Q1",Value(0.0001)).asDouble();
        	diagQ[1]=rf.check("Q2",Value(0.0001)).asDouble();
        	diagQ[2]=rf.check("Q3",Value(0.0001)).asDouble();
        	diagQ[3]=rf.check("Q4",Value(0.001)).asDouble();
        	diagQ[4]=rf.check("Q5",Value(0.001)).asDouble();
        	diagQ[5]=rf.check("Q6",Value(0.001)).asDouble();
		}
      
        yarp::sig::Vector diagR;
        diagR.resize(parameters.p,1);
        
        check=readDiagonalMatrix("R",diagR,parameters.p);

		if(!check)
		{
        	diagR[0]=rf.check("R1",Value(0.0001)).asDouble();
        	diagR[1]=rf.check("R2",Value(0.0001)).asDouble();
        	diagR[2]=rf.check("R3",Value(0.0001)).asDouble();
		}
		
        yarp::sig::Vector diagP0;
        diagP0.resize(parameters.n,1);
        
        check=readDiagonalMatrix("P0",diagP0,parameters.n);
	
		if(!check)
		{
        	diagP0[0]=rf.check("P01",Value(0.04)).asDouble();
        	diagP0[1]=rf.check("P02",Value(0.04)).asDouble();
        	diagP0[2]=rf.check("P03",Value(0.04)).asDouble();
        	diagP0[3]=rf.check("P04",Value(pow(M_PI,2.0))).asDouble();
        	diagP0[4]=rf.check("P05",Value(pow(M_PI/2.0,2.0))).asDouble();
        	diagP0[5]=rf.check("P06",Value(pow(M_PI,2.0))).asDouble();
		}
		
		
		yarp::sig::Vector neigh0;
        neigh0.resize(parameters.n,1);
        
        check=readDiagonalMatrix("neigh",neigh0,parameters.n);
	
		if(!check)
		{
        	neigh0[0]=rf.check("neigh0",Value(0.08)).asDouble();
        	neigh0[1]=rf.check("neigh1",Value(0.08)).asDouble();
        	neigh0[2]=rf.check("neigh2",Value(0.08)).asDouble();
        	neigh0[3]=rf.check("neigh3",Value(0.3)).asDouble();
        	neigh0[4]=rf.check("neigh4",Value(0.3)).asDouble();
        	neigh0[5]=rf.check("neigh5",Value(0.3)).asDouble();
		}
		
		
		parameters.neigh=neigh0;
		
		 cout<<"perc "<<parameters.percent<<endl;
        cout<<"neigh "<<parameters.neigh.toString(6,3)<<endl;
		
       
        yarp::sig::Matrix Q;
        Q.resize(parameters.n,parameters.n);
       
        yarp::sig::Matrix R;
        R.resize(parameters.p,parameters.p);
       
        yarp::sig::Matrix P0;
        P0.resize(parameters.n,parameters.n);
        
 		for(size_t i=0; i<parameters.n; i++)
	    {
	          
	         Q(i,i)=diagQ[i];
	         P0(i,i)=diagP0[i];
	    }
	    parameters.Q=Q;
	    parameters.P0=P0;
	   
	      
	    for(size_t i=0; i<parameters.p; i++)
	    {
	         R(i,i)=diagR[i];
	        
	    }

        parameters.R=R;     
      
        // read the polyhedron from a .OFF file
         ifstream modelFile(modelFileName.c_str());
        if (!modelFile.is_open())
        {
            yError()<<"problem opening model file!";
            return false;
        }
        
        modelFile>>get_model();
        
        
        if (modelFile.bad())
        {
            yError()<<"problem reading model file!";
            modelFile.close();
            return false;
        }
        modelFile.close();

        // read the measurements file
        ifstream measurementsFile(measurementsFileName.c_str());
        if (!measurementsFile.is_open())
        {
            yError()<<"problem opening measurements file!";
            modelFile.close();
            return false;
        }
        if (!readMeasurements(measurementsFile))
        {
            yError()<<"problem reading measurements file!";
            modelFile.close();
            measurementsFile.close();
            return false;
        }
        measurementsFile.close();
        for(int i=0;  i<measurements.size(); i++)
        cout<<"measurements "<< measurements[i]<<endl;
        
          parameters.window_width=rf.find("window_width").asInt();
        if (rf.find("window_width").isNull())
        parameters.window_width=rf.check("window_width",Value(parameters.numMeas)).asInt();
        cout<<"WINDOW_WIDTH"<<parameters.window_width<<endl;
 
}

/*******************************************************************************/
void UnscentedParticleFilter::saveData(const yarp::sig::Vector &ms_particle)
{    
	string outputFileName=this->rf->check("outputFileMUPF",Value("../../outputs/outputMUPF.off")).
                       asString().c_str();
    string  outputFileName2=this->rf->check("outputFileDataMUPF",Value("../../outputs/output_dataMUPF.off")).
                       asString().c_str();
                     
                       
	ofstream fout(outputFileName.c_str());
    if(fout.is_open())
    {
        fout<<get_model();
 
    }
    else
		cout<< "problem opening output_data file!";
		
	fout.close();
	        
    ofstream fout2(outputFileName2.c_str());                                           

	if(fout2.is_open())
	{   
		fout2<<"highest weight "<<endl;
		fout2 << "s: "<<ms_particle1.pos.subVector(0,5).toString(3,3).c_str()<<endl;
  	    fout2 << "found in "<<result[7]<<" [s]"<<endl;
	    fout2<< "error_index "<<ms_particle1.error_index<<endl;
	    
	    fout2<<"highest density with all particles"<<endl;
		fout2 << "solution: "<<ms_particle2.pos.subVector(0,5).toString(3,3).c_str()<<endl;
		fout2 << "found in "<<result[8]<<" [s]"<<endl;
	    fout2<< "error_index "<<ms_particle2.error_index<<endl;
	    
	    fout2<<"highest density with neighborhood"<<endl;
		fout2 << "solution: "<<ms_particle3.pos.subVector(0,5).toString(3,3).c_str()<<endl;
		fout2 << "found in "<<result[9]<<" [s]"<<endl;
	    fout2<< "error_index "<<ms_particle3.error_index<<endl;
	
	}
	else
	    cout<< "problem opening output_data file!";

     fout2.close();  
}

void UnscentedParticleFilter::saveStatisticsData(const yarp::sig::Matrix &solutions)
{
	string outputFileName2=this->rf->check("outputFileMUPF",Value("../../outputs/outputStatisticsMUPF.off")).
                       asString().c_str();
                       
     double average1,average2,average3;   
     average1=0;
     average2=0;
     average3=0;
                       
    ofstream fout2(outputFileName2.c_str());                                           

	if(fout2.is_open())
	{
		for(int j=0; j<solutions.rows(); j++)
		{
		
				fout2<<"trail "<<j<<": "<<solutions.getRow(j).toString(3,5).c_str()<<endl;
				average1=average1+solutions(j,0);
				average2=average2+solutions(j,1);
				average3=average3+solutions(j,2);
				
		
		}
		
		fout2<<"average "<< average1/solutions.rows()<<" "<<average2/solutions.rows()<<" "<<average3/solutions.rows()<<" "<<endl;
	}
}
       
/*******************************************************************************/      
yarp::sig::Vector UnscentedParticleFilter::localization()
{
    yarp::sig::Vector result;
  
   init();
   solve();
   return result=finalize();
        
};

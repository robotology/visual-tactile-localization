/** May 2015- Univerity of Florence
 *  @author Giulia Vezzani
 *  Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */ 

#ifndef UPFINTERFACE_H
#define UPFINTERFACE_H

#include "localizer.h"
#include "geometryCGAL.h"
#include <string>
#include <iostream>
#include <fstream>
#include <yarp/sig/all.h>
#include <yarp/os/ResourceFinder.h>

/*******************************************************************/
struct ParametersUPF : public Parameters
{
    /**number of particles*/
    int N;
    
    /** width of the window to compute likelihood */
    int window_width;
    
    /**number of measurement for measurementFile*/
    int numMeas;
	
    /**number of degree of freedom*/
    int n;
	
    /**dimension of measurements*/
    int p;
   
    /** Unscented Transform parameters*/
    double            beta;
    double            kappa;
    double            alpha;
    double           lambda;
    double           percent;
    
    /** Covariance matrix of process noise Q, measurement noise R and of initial estimate*/
    yarp::sig::Matrix Q;
    yarp::sig::Matrix R;
    yarp::sig::Matrix P0;
    
    /**  Dimension of initial area*/
    yarp::sig::Vector center0;
    yarp::sig::Vector radius0;  
    yarp::sig::Vector neigh;
       
};

/*******************************************************************/
struct ParticleUPF
{
    /** Particles after correction step*/
    yarp::sig::Vector x_corr;
    
    /** Auxiliary matrix for operations*/
    yarp::sig::Matrix x_tilde;
    yarp::sig::Vector x_bar;
    yarp::sig::Matrix A;
    yarp::sig::Matrix P_pred_aux;
    
    /** Prediction measurements */
    yarp::sig::Vector y_pred;
    
    /** weights*/ 
    double            weights;
    
    /** Matrices of correction and predicion steps*/
    yarp::sig::Matrix P_corr;
    yarp::sig::Matrix P_hat;
    yarp::sig::Matrix P_pred;
    
    /** Matrice, Pyy and Pxx, for compute correction gain K*/
    yarp::sig::Matrix Pyy;
    yarp::sig::Matrix Pxy;
    yarp::sig::Matrix K;
    
    /** Predicted particles */
    yarp::sig::Vector x_pred;
    


    /** Sigma Points for average and covariance*/
    yarp::sig::Matrix XsigmaPoints_corr;
    yarp::sig::Matrix XsigmaPoints_pred;
    yarp::sig::Matrix YsigmaPoints_pred;
    yarp::sig::Vector WsigmaPoints_average;
    yarp::sig::Vector WsigmaPoints_covariance;
    
    /** Particle initializaion
     */
    ParticleUPF() : x_corr(6,0.0),
                    x_pred(6,0.0),
                    x_tilde(6,1),
                    x_bar(6,0.0),
                    y_pred(3,0.0),
                    P_corr(6,6),
                    P_hat(6,6),
                    P_pred(6,6),
                    Pyy(3,3),
                    Pxy(6,3),
                    K(6,3),
                    A(3,1),
                    P_pred_aux(6,6),
                    XsigmaPoints_corr(6,13),
                    XsigmaPoints_pred(6,13),
                    YsigmaPoints_pred(3,13),
                    WsigmaPoints_average(13,0.0),
                    WsigmaPoints_covariance(13,0.0),
                    weights(std::numeric_limits<double>::infinity()) { }
};

/*******************************************************************/

struct MsParticleUPF
{
    /** average of most significant particle*/
    yarp::sig::Vector pos;
    
    /** error index associated to most significant particle*/
    double            error_index;
    
    /** MsParticle initialization 
     */
    MsParticleUPF() : pos(6,0.0),
                 error_index(std::numeric_limits<double>::infinity()) { }
};

/*******************************************************************/

/** This class is the mplementation of the Memory Unscented Particle Filter
 * (MUPF), for 6d localization problem
 * It works with 3d object model, consisting of triangular meshes, in .off
 * format 
 * This class is derived from  abstract class localizator and from  abstract 
 * class optimizer The last one is necessary to use CGAL for distances calculus
 */
class UnscentedParticleFilter : public GeometryCGAL, public Localizer
{
    /** all particles are  in x
     * to each particle all matrixes and vectors for UPF are associated
     */
    std::deque<ParticleUPF> x;
    
    /** most significatn particle*/
    ParticleUPF            ms_particle;
    
    /** This is a point of the current measurements*/
    std::deque<Point>  current_measurement;
    
    /** This is an auxiliary vector*/
    std::deque<yarp::sig::Vector> x_most;
    
    /** index of current measurements
     */
    int t;
    
    /** degree of freedom
     */
    int n;

    /** dimension of measurements
    */
    int p;
	
protected:
 
    /** starting time*/
    double t0;
    
    /** executional time*/
    double dt;
    
     /** vector for solution with high weight*/
     MsParticleUPF ms_particle1;
    
    /** vector for solution with high density*/
     MsParticleUPF ms_particle2;
     
      /** vector for solution with high density second method*/
     MsParticleUPF ms_particle3;
     
      /** vector for solution with high density with sum of gaussians*/
     MsParticleUPF ms_particle4;
     
     
     yarp::sig::Vector result;
     
     yarp::sig::Vector result4;

     /** For downsampling*/
     int down;

   
    /*******************************************************************/
    /** Get parameters necessary for UnscentedParticleFilter class
    */
    ParametersUPF &get_parameters();

    /*******************************************************************/     
    /** Init Optimizer, set starting time=0, sample particle from initial
    * search region, initialize matrix and vector of UPF
    */
    void init();
    
    /*******************************************************************/   
    /** Sample particle from initial search region,
    * specified by radius0 and center0. Radius0 and center0 are set by default to 0.2 and 0.2 m.
    * They can be changed throw command line or in a config.ini
    */
    void initialRandomize();
    
    /*******************************************************************/   
    /** It is a single iteration  of UPF
    * @return true when UPF is completed
    */
    bool step();
   
    /*******************************************************************/   
    /** It is executed when UPF is finished.
    *  Compute final most significant particle, in world reference system.
    * @return most significant particle, as yarp Vector
    */ 
    yarp::sig::Vector finalize();
    
    /*******************************************************************/ 
    /** Compute roll pitch roll rototranslation matrix.
    * @param particle, whose first three components are the coordinate 
    *         of origin of the reference system, the last three the
    *         representation of its orientation with Euler angles, zyz
    * @return the yarp Matrix of rototranslation 
    */
    yarp::sig::Matrix rpr(const yarp::sig::Vector &particle);
   
    /*******************************************************************/
    /**Return predicted measurements, as closest point of object to measurements. 
    * Object is in pose represented by sigma point j of Unscented Transform of particle k
    * @param t index of current measurement
    * @param  k current particle
    * @param j current sigma point
    * @return predicted measurement, as yarp Vector
    */ 
    yarp::sig::Vector compute_y(const int &t,const  int &k,const  int &j);
    
     /*******************************************************************/ 
  
    /** Return likelihood (probability to have a measurement y, given object pose x)
    * @param t index of current measurements
    * @param k current particle
    * @return value of likelihood
    */
    double likelihood(const int &t, const int &k);
    
    /*******************************************************************/   
    /** Realize random resampling: particle with low weights are suppresed
    * Particle with high weights are duplicated
    */
    void resampling();
    
     /*******************************************************************/ 
    /** Compute Sigma Points for i-th particle
    */
    void computeSigmaPoints(const int &i);
    
    /*******************************************************************/
    /** Compute performance index (average among measurementsof minimum
    * distance of each measurement to object) and ms_particle.error_index 
    * is modified
    * @param ms_particle is the most significant particle,ms_particle.pos is
    * used to compute performancce index that is saved in ms_particle.error_index
    */
    void performanceIndex(MsParticleUPF &ms_particle);
  
    /*******************************************************************/
    /** Executes prediction step of Unscented Kalman Filter for each particle
    * @param i current particle
    */
    void predictionStep(const int &i);
    
    /*******************************************************************/
    /**Initializes all matrices  necessary for UKF step for each particle
    *  @param i current particle
    */
    void initializeUKFMatrix(const int &i);
    
    /*******************************************************************/
    /** Compute prediciton matrix for each particle in prediction step
    * @param i current particle
    */
    void computePpred(const int &i);
    
    /*******************************************************************/
     /** Compute correction matrix for each particle in prediction step
     * @param i current particle
     */
     void computeCorrectionMatrix(const int &i);
    
     /*******************************************************************/
     /** Compute correction average, for each particle,using current measurement
     * @param i current particle
     */
     void correctionStep(const int &i);
   
    /*******************************************************************/
     /** Compute weights for particle i, usign all measurements collected until 
     * current iterations and compute their sum
     * @param i current particle
     * @param sum sum of weights
     */
     void computeWeights(const int &i, double &sum);
    
    /*******************************************************************/
     /** Normalize weights for particle i using sum and compute the sum of square weights
     * @param i current particle
     * @param sum sum of weights
     * @params sum_squared sum of squared weights
     */
     void normalizeWeights(const int &i, const double &sum, double &sum_squared);
   
    /*******************************************************************/
    /** Find particle with highest weights 
    */
    void findMostSignificantParticle();
    
    /*******************************************************************/
     /** If we are at third and successive measurement and if Neff=1/sum_squared
     * is minor than N(number of particle)/20 resampling() is called
     * otherwise weights are set to 1/N
     * @param Neff 1/sum_squared
     * @param sum_squared sum of squared weights
     */
     void selectionStep(double &Neff,const double &sum_squared);
    
     /*******************************************************************/
     /** Initialize other vectors and parameters of UPF
     */
      void initializationUPF();
    
     /*******************************************************************/
     /** Read measurements froma  text file
     * @param fin is the ifstream associated to measurements file
     * @return true/false on succes/failure
     */
     bool readMeasurements(std::ifstream &fin, const int &down);
    
     /*******************************************************************/
     /** Find the particle with the highest density
     */
     yarp::sig::Vector particleDensity();
     /*******************************************************************/
     
     
     /** Find the particle with the highest density
   
     */
     yarp::sig::Vector particleDensity2();
     /*******************************************************************/
     
     /** Find the particle with the highest rpobability
   
     */
     yarp::sig::Vector particleDensity3();
     /*******************************************************************/
     
      /**Time with the new method
   
     */
     double dt_gauss;
       /*******************************************************************/
          /**Time with the new method
   
     */
     double dt_gauss2;
      /*******************************************************************/
      double DT;
     
    
public:
    
    
     /** constructor, derived from optimizer
     */
     UnscentedParticleFilter();
    
    
     /** Calls step( ) until measurements finish
     */
     void solve();
    
     /*******************************************************************/
    
     /** Saves x with gretest weight, localization error and execution time 
     * in one file, and the .off model of the estimated pose object in another file 
     * @param rf a previously inizialized @see Resource Finder
     * @param ms_particle, containing estimated x, localization error and execution time
     */
     void saveData( const yarp::sig::Vector &ms_particle,const int &y);
     
     /*******************************************************************/
    
     /** Saves the solution and the time computed for all the trials, computes the average
     * of the solution and of the time of the trials 
     * @param Matrix, containing the three solutions
     */
     void saveStatisticsData(const yarp::sig::Matrix &solutions);
    
     /*******************************************************************
     /**Configures all parameters needed by the algorithm, reading them from a 
     * configuration file .ini or from command line. If the user doesn't provide
     * them, they are set to default values
     * @param rf a previously inizialized @see Resource Finder
     * @return true/false on succes/failure
     */
     bool configure(yarp::os::ResourceFinder &rf, const int &i);
     /*******************************************************************/
    
     /** Runs init() and solve(). Configure must be runned before and if we want to
     * save or to send the result, we have to do it in main.cpp
     * @param rf a previously inizialized @see Resource Finder
     * @return vector of 8 components: estimated x,y,z, and the 3 Euler angles,
     * localization error, execution time
     */
     yarp::sig::Vector localization();     
     /****************************************************************************************************/

   
};


#endif

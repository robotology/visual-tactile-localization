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

#ifndef UNSCENTED_PARTICLE_FILTER_H
#define UNSCENTED_PARTICLE_FILTER_H

// std
#include <string>

// yarp
#include <yarp/sig/all.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/math/RandnScalar.h>

// CGAL
#include "geometryCGAL.h"

typedef std::vector<Point> Measure;

/**
 * Struct that contains the parameters of the Unscented Particle Filter.
 */
struct ParametersUPF
{
    // number of particles
    int N;
    
    // number of DoFs
    int n;

    // threshold for check on Neff
    double n_eff_thr;

    // number of steps to wait for
    // before checking for Neff and possibly use
    // the resampling mechanism
    int n_steps_before_resampling;

    // whether to use the ideal measurement equation or not
    bool use_ideal_meas_eqn;
   
    // Unscented Transform parameters
    double beta;
    double kappa;
    double alpha;
    double lambda;
    double percent;
    
    // covariance matrix of system noise Q
    yarp::sig::Matrix Q;
    //
    yarp::sig::Matrix Q_prev;
    // scalar measurement noise variance R
    double R;
    // coavariance matrix of initial guess P0
    yarp::sig::Matrix P0;
    
    // size of the initial research area
    yarp::sig::Vector center0;
    yarp::sig::Vector radius0;  
    yarp::sig::Vector neigh;
};

/**
 * Struct that contains the quantities related to each particle.
 */
struct ParticleUPF
{
    // weights
    double weights;
    double prev_weights;

    // particle after correction step
    yarp::sig::Vector x_corr;
    // previoust value of x_corr
    yarp::sig::Vector x_corr_prev;    
    // predicted state
    yarp::sig::Vector x_pred;
    // predicted measurement
    yarp::sig::Vector y_pred;
    
    // estimated state covariance matrix
    yarp::sig::Matrix P_corr;
    // predicted state covariance matrix
    yarp::sig::Matrix P_pred;
    // predicted measurement covariance matrix
    yarp::sig::Matrix Pyy;
    // predicted state-measurement covariance matrix
    yarp::sig::Matrix Pxy;

    // auxiliary quantities
    yarp::sig::Matrix x_tilde;
    yarp::sig::Vector x_bar;
    yarp::sig::Matrix A;
    yarp::sig::Matrix P_pred_aux;
    
    // UKF Kalman gain
    yarp::sig::Matrix K;
    
    // Sigma Points for Unscented Transformation
    yarp::sig::Matrix XsigmaPoints_corr;
    yarp::sig::Matrix XsigmaPoints_pred;
    yarp::sig::Matrix YsigmaPoints_pred;
    yarp::sig::Vector WsigmaPoints_average;
    yarp::sig::Vector WsigmaPoints_covariance;
    
    // initialization
    ParticleUPF() : x_corr(6,0.0),
	            x_corr_prev(6,0.0),
	            x_pred(6,0.0),
	            x_tilde(6,1),
	            x_bar(6,0.0),
	            P_corr(6,6),
	            P_pred(6,6),
	            P_pred_aux(6,6),
	            XsigmaPoints_corr(6,13),
	            XsigmaPoints_pred(6,13),
	            WsigmaPoints_average(13,0.0),
	            WsigmaPoints_covariance(13,0.0),
	            weights(std::numeric_limits<double>::infinity()),
	            prev_weights(std::numeric_limits<double>::infinity()) { }
};

/** 
 *  This class is the implementation of the Memory Unscented Particle Filter (MUPF)
 *  that solves the 6D localization problem.
 *
 *  It requires the 3D object model of the object to be localized. 
 *  The model is provided as a triangular mesh in a .OFF (Object File Format) file.
 *
 *  This class is derived from the abstract class GeometryCGAL required to 
 *  evaluate distances from a measurement to the object using the CGAL library.
 */
class UnscentedParticleFilter : public GeometryCGAL
{
private:
    // storage for all the particles
    std::deque<ParticleUPF> x;
    
    // current measure
    Measure curr_meas;
    
    // current real pose
    yarp::sig::Vector real_pose;

    // system input
    yarp::sig::Vector prev_input;
    yarp::sig::Vector new_input;
    yarp::sig::Vector propagated_input;

    // parameters of the UPF
    ParametersUPF params;

    // index of current iteration
    int t;

    // previous filtering time
    double t_prev;

    // current estimate
    yarp::sig::Vector current_estimate;

    /** 
     * Read the coordinates of the center of the research region 
     * and save them in center0.
     * @param rf reference to a previously instantiated @see ResourceFinder
     * @param the tag of the center within the internal resource finder rf
     * @param vector in which coordinates are returned
     * @return true/false upon succes/failure
     */
    bool readCenter(const yarp::os::ResourceFinder &rf,
		    const std::string &tag,
		    yarp::sig::Vector &center0);
    
    /** 
     * Read the coordinates of the radius of the research region and save them in radius0.
     * @param rf reference to a previously instantiated @see ResourceFinder
     * @param tag the tag of radius within the internal resource finder rf
     * @param radius0 vector in which coordinates are returned
     * @return true/false upon succes/failure
     */
    bool readRadius(const yarp::os::ResourceFinder &rf,
		    const std::string &tag,
		    yarp::sig::Vector &radius0);
    
    /** 
     * Read the values on the diagonal line of a diagonal matrix 
     * stored as tag within the internal resource finder rf and 
     * store them in the vector diag.
     * @param rf reference to a previously instantiated @see ResourceFinder
     * @param tag the tag of the matrix in the internal resource finder rf
     * @param diag the vector in which values are returned
     * @param dimension the dimension of the matrix
     * @return true/false upon succes/failure
     */
    bool readDiagonalMatrix(const yarp::os::ResourceFinder &rf,
			    const std::string &tag,
			    yarp::sig::Vector &diag,
			    const int &dimension);

    /** 
     * Initialize some quantities of the UPF
     */
    void initializationUPF();

    /**
     * Sample particles from the initial search region.
     * The size of the region can be specified using the parameters 'radius0'
     * and 'center0' in a .ini configuration file or in the command line arguments.
     */
    void initialRandomize();

    /** 
     * Resize matrices and vectors, within a particle,
     * whose size depends on the size of the current measurement vector.
     * @param i index of the current particle
     */
    void resizeParticle(const int &i);

    /**
     * Initialize the matrices required within the UKF step of the UPF
     * @param i index of the current particle
     */
    void initializeUKFMatrix(const int &i);

    /** 
     * Compute Sigma Points for the i-th particle
     */
    void computeSigmaPoints(const int &i);

    /**
     * Compute a rototranslation matrix. 
     * Angles should be provided as Euler ZYZ.
     * @param pose yarp::sig::Vector containing the position and attitude
     * @return a rototranslation matrix
     */
    yarp::sig::Matrix homogeneousTransform(const yarp::sig::Vector &pose);
        
    /**
     * Compute the predicted measurement as the vector of closest point(s)
     * of the object to the measurement.
     * 
     * The object is in the pose represented by the sigma point j
     * of the particle k.
     *
     * @param k index of the current particle
     * @param j index of the current sigma point
     * @return predicted measurement as a yarp::sig::Vector
     */
    yarp::sig::Vector computeY(const int &k, const int &j);

    /**
     * Compute the predicted measurement as the vector of point(s)
     * origintaing from the contact with the surface of the object
     * as if the contact takes place with the object in the pose
     * represented by the sigma point j of the particle k.
     *
     * @param k index of the current particle
     * @param j index of the current sigma point
     * @return ideal predicted measurement as a yarp::sig::Vector
     */
    yarp::sig::Vector computeYIdeal(const int &k, const int &j);

    /** 
     * Execute the prediction step required within the UPF.
     * @param i index of the current particle
     */
    void predictionStep(const int &i);
    
    /**
     * Compute the predicted state covariance matrix.
     * @param i index of the current particle
     */
    void computePpred(const int &i);
    
    /** 
     * Compute matrices required to correct the predicted state
     * with the measurements.
     * @param i index of the current particle
     */
    void computeCorrectionMatrix(const int &i);

    /** 
     * Execute the UKF correction step.
     * @param i index of the current particle
     */
    void correctionStep(const int &i);
    
    /** 
     * Compute the likelihood required within the UPF.
     * @param k index of the current particle
     * @param map_likelihood value of likelihood with MAP correction
     * @return the value of the likelihood
     */
    double likelihood(const int &k, double& map_likelihood);

    /** 
     * Eval a multivariate gaussian.
     * @param x a yarp::sig::Vector containing the argument
     * @param mean a yarp::sig::Vector containing the mean
     * @param covariance a yarp::sig::Matrix containing the covariance
     * @return the value of the density.
     */
    double multivariateGaussian(const yarp::sig::Vector &x,
				const yarp::sig::Vector &mean,
				const yarp::sig::Matrix &covariance);
    /** 
     * Compute the transition probability.
     * @param i the index of the particle to consider for the current state
     * @param j the index of the particle to consisder for the previous state
     * @return the transition probability
     */
    double tranProbability(const int &i, const int &j);
    
    /** 
     * Compute weight for particle i and 
     * update the sum of all the weights.
     * @param i index of the current particle
     * @param sum sum of the weights
     */
    void computeWeights(const int &i, double &sum);
    
    /** 
     * Normalize weight for particle i using the sum and 
     * compute the sum of squared weights.
     * @param i index of the current particle
     * @param sum sum of the weights
     * @params sum_squared sum of the squared weights
     */
    void normalizeWeights(const int &i, const double &sum, double &sum_squared);
    
    /** 
     * Execute the resampling step required within the UPF.
     */
    void resampling();
   
    /** 
     * Execute resampling if the iteration index is greater 
     * than 3 and if the criterion based on Neffective is satisfied.
     * Otherwise weights are set to 1/N.
     *
     * If get_parameters().always_resample is true only the 
     * criterion based on Neffective is considered.
     * 
     * @param sum_squared sum of squared weights
     */
    void selectionStep(const double &sum_squared);
             
public:
    /**
     * Configures all parameters needed by the algorithm.
     * @param rf a previously inizialized @see Resource Finder
     * @return true/false on succes/failure
     */
    bool configure(yarp::os::ResourceFinder &rf);

    /** 
     * Init the CGAL distance computation engine, 
     * set the iteration index to 0 and store the current time,
     * sample particles from the initial research region
     * and initialize some matrices and vectors of UPF.
     */
    void init();

    /**
     * Provide a new system input to the algorithm.
     * @param in a yarp::sig::Vector containing the input
     */
    void setNewInput(const yarp::sig::Vector &in);
    
    /**
     * Provide a new measurement to the algorithm.
     * @param m a std::vector of yarp::sig::Vector points
     */
    void setNewMeasure(const std::vector<yarp::sig::Vector>& m);

    /**
     * Provide the current real position.
     * @param pose a yarp::sig::Vector containing the real pose
     */
    void setRealPose(const yarp::sig::Vector &pose);

    /**
     * Set the system noise covariance matrix.
     * @param covariance a yarp::sig::Vector containing the diagonal
     * entries of the matrix Q
     */
    void setQ(const yarp::sig::Vector &covariance);

    /**
     * Set the scalar measurement noise variance R
     * @param variance the value of the scalar measurement noise variance
     */
    void setR(const double &variance);

    /**
     * Reset the internal time in order to handle properly
     * the propagation of the nominal model.
     */
    void resetTime();

    /**
     * Skip the next filtering step.
     */
    void skipStep();
    
    /**
     * Single iteration of the algorithm.
     */
    void step();

    /** 
     * Get the MAP estimate.
     */
    yarp::sig::Vector getEstimate();

    /** 
     * Compute the MAP estimate.
     */
    void evalEstimate();

    /** 
     * Compute the performance index
     * (average of distances between a given set of contact points 
     * and the object in a given estimated pose).
     *
     * @param estimate the estimate found by the algorithm
     * @param points a std::deque<Point> of Point points
     * @return the performance index
     */
    double evalPerformanceIndex(const yarp::sig::Vector &estimate,
				const std::deque<Point> &points);
    
    /** 
     * Transform the model of the object with coordinates 
     * expressed in object fixed frame into the model of the object
     * in a given estimated pose with coordinates expressed in robot
     * frame.
     *
     * @param estimate the estimate found by the algorithm
     * @param transformed the transformed object model
     * @return the transformed model as a Polyhedron
     */
    void transformObject(const yarp::sig::Vector &estimate,
			 Polyhedron &transformed);
};

#endif

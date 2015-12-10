
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

#ifndef SCALING_H
#define SCALING_H
#include "geometryCGAL.h"
#include "localizer.h"

#include <deque>
#include <yarp/sig/all.h>
#include <yarp/os/ResourceFinder.h>



/*******************************************************************************/

struct ParametersSSprune : public Parameters
{
    /**number of particles*/
    int               numParticles;
    
    /**number of measurement for measurementFile*/
	int               numMeas;
	
	/**number of degree of freedom*/
	int               n;
	
	/**dimension of measurements*/
    int               p;
	
   	/** standard deviation for contact point measurement noise*/
    double            errp;
    
    /** standard deviation for surface normal measurement noise*/
    double            errn;
    
    /** desired precision*/
	double            delta_desired_p;
	double            delta_desired_n;
	
	/** Dimension of initial area*/
	yarp::sig::Vector center0;
    yarp::sig::Vector radius0;  

};


/*******************************************************************************/

struct Particle
{  
    /** pose of object*/
    yarp::sig::Vector pos;
    
    /** weight associated*/
    double            weights;
    
    /** initialization for particle */
    Particle() : pos(6,0.0),
                 weights(std::numeric_limits<double>::infinity()) { }
};

/*******************************************************************************/

struct MsParticle
{
    /** pose of most significant particle*/
    yarp::sig::Vector pos;
    
    /** error index associated to most significant particl*/
    double            error_index;
   
    /** initialization for MSparticle */
    MsParticle() : pos(6,0.0),
                 error_index(std::numeric_limits<double>::infinity()) { }
};


/*******************************************************************************/

/** This class is the mplementation of the ScalingSeries algorithm
 * for 6d localization problem
 * It works with 3d object model, consisting of triangular meshes, in .off
 * format 
 * This class is derived from  abstract class localizator and from  abstract 
 * class optimizer The last one is necessary to use CGAL for distances calculus
 */
class ScalingSeries : public GeometryCGAL, public Localizer
{
     /** all particles are represented in x
     */
     std::deque<Particle> x;
    
     /** most significant particle */
     Particle             ms_particle;
    
     /** a point for current measurement */
     std::deque<Point>  current_measurement;
    
     /** current iteration*/
     int iter;
   
   	 /** degree of freedom*/ 
    	int n;

     /** dimension of measurement*/
   	 int p;
	
    /** number of iterations*/
    int N;
	
	/**auxiliary variable*/
	int count_meas;
	
	/** current number of particle*/
    int numTotPart;
	
	/** zoom factor that is chosen to that volume is halved*/
    double zoom;
	
	/** vector to save number of particles durign iterations */
	yarp::sig::Vector number;

    /** current values of delta and radius of spheres*/
	double delta_p,delta_n, radius_p, radius_n;

    protected: 

     /** starting time*/
     double t0;
    
     /** executional time*/
     double dt;

     /*******************************************************************************/
   
     /** Get parameters necessary for ScalingSeries class*/
     ParametersSSprune &get_parameters();
    
     /*******************************************************************************/
    
     /** initialize optimizer, get parameters
     *  initialize number of iterations and zoom factor
     *  compute number of iterations
     */
     void init();
    
     /*******************************************************************************/
   
     /** Return volume of a sphere of n dimensions of a given radius
     * @param radius radius of the sphere
     * @param n dimension of the sphere
     * @return volume of sphere
     */
     double volumeSphere(const double &radius,const int &n);
    
     /*******************************************************************************/
  
     /** Sample particle from initial search region,
     * specified by radius0 and center0. Radius0 and center0 are set by default to 0.2 and 0.2 m.
     * They can be changed throw command line or in a config.ini
     */
     void initialRandomize();
   
     /*******************************************************************************/
     
     /** It is the execution of Scaling Series
     */
     void solve();
    
     /*******************************************************************************/
    
     /** It is executed when Scaling Series is finished.
     *  Compute final most significant particle, in world reference system.
     * @return most significant particle, as yarp Vector
     */ 
     yarp::sig::Vector finalize();
    
     /*******************************************************************************/
    
     /** Compute roll pitch roll rototranslation matrix.
     * @param particle, whose first three components are the coordinate 
     *         of origin of the reference system, the last three the
     *         representation of its orientation with Euler angles, zyz
     * @return the yarp Matrix of rototranslation 
     */
     yarp::sig::Matrix rpr(const yarp::sig::Vector &particle);
    
     /*******************************************************************************/
   
     /** Sample particle so that density of particles is constant in each sphere
	 * paying attention that a particle belong to only one sphere
	 * @param numTotPart is the number of sampled particles ( change at each iteration)
	 */
	 void evenDensityCover(int &numTotPart);
	
	 /*******************************************************************************/
	
	 /** Check if theres is new sampled particle belong to spheres of previous particle until i-th one
     * @param i current particle
     * @param new_x auziliary particle to make sampling and checking in correct way
     */
     bool belongToOtherSpheres(const int &i, const Particle &new_x);
    
     /*******************************************************************************/
   
     /** Compute weights using all mesurements, with annealed data probability
	 * using tau
	 * @param tau anneling paramter
	 * @param numTotPart current number of particles
	 */
	 void computeWeights(const double &tau_p, const int &numTotPart);
	
	    /*******************************************************************************/
	
	 /** Cut off particles with low weights, without dulicate the ones with high weights
     * number of particle changes
     * @param numTotPart current number of particles
	 */
	 void pruning(int& numTotPart);
	
	 /*******************************************************************************/	
	 
     /** Check if two number are equal, unless a tollerance
     * @param x,y number to be compared
     * @return true if their difference is less than a tollerence
     */
     bool isEqual( const double &x, const double &y);
    
     /*******************************************************************************/
   
     /** Compute performance index ( average among measurements
     * of minimum distance of each measurement to object) and 
     * ms_particle.error_index is modified
     * @param ms_particle is the most significant particle.
     *         ms_particle.pos is used to compute performancce index 
     *         that is saved in ms_particle.error_index
     */
     void performanceIndex(MsParticle &ms_particle);
    
     /*******************************************************************************/
  
     /** It is the execution of one iteration of Scaling Series
     * @return true when it is completed
     */
     bool step();
   
     /*******************************************************************************/
   
     /** Read measurements froma  text file
     * @param fin is the ifstream associated to measurements file
     * @return true/false on succes/failure
     */
     bool readMeasurements(std::ifstream &fin);
    
     /*******************************************************************************/
  
     public:  
  
  
     /** ScalingSeries constructor */
     ScalingSeries();
    
     /*******************************************************************************/
     
     /** Saves x with gretest weight, localization error and execution time 
     * in one file, and the .off model of the estimated pose object in another file 
     * @param rf a previously inizialized @see Resource Finder
     * @param ms_particle, containing estimated x, localization error and execution time
     */
     void saveData(const yarp::sig::Vector &ms_particle);
    
     /*******************************************************************************/
    
     /**Configures all parameters needed by the algorithm, reading them from a 
     * configuration file .ini or from command line. If the user doesn't provide
     * them, they are set to default values
     * @param rf a previously inizialized @see Resource Finder
     * @return true/false on succes/failure
     */
     bool configure(yarp::os::ResourceFinder &rf);
     
     /*******************************************************************************/ 
     
     /** Saves the solution and the time computed for all the trials, computes the average
     * of the solution and of the time of the trials 
     * @param Matrix, containing the three solutions
     */
     void saveStatisticsData(const yarp::sig::Matrix &solutions);
     /*******************************************************************************/ 

      
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

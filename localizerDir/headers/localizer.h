
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
 
#ifndef LOCALIZATOR_H
#define LOCALIZATOR_H

#include <iostream>
#include <string>
#include <yarp/sig/all.h>
#include <yarp/os/ResourceFinder.h>



/**Allows to localize a 3D object using tactile sensors. This is an abstract base class.
 * It provides some virtual methods and some pure virtual methods.
 * It can be used to achieve offline or online localization with a generic 
 * localization algorithm, saving the results on files or sending them through a port.
 * The not pure methods are define to work in the case in which the problem is a problem of 6D
 * localization and pose object is represented by x,y,z and three Euler angle (roll pitch roll)
 */
class Localizer
{
    
    protected:
    
    yarp::os::ResourceFinder *rf;
    
    /*******************************************************************/
    
    /** read the coordinates of the center of the research region and save them in center0
     * @param rf a previously inizialized @see Resource Finder
     * @param the tag of center in an input file
     * @param vector in which coordinates are saved
     * @return true/false upon succes/failure
     */
    virtual bool readCenter(const std::string &tag, yarp::sig::Vector &center0);
    
    /*******************************************************************/
    
    /** read the coordinates of the radius of the research region and save them in radius0
     * @param rf a previously inizialized @see Resource Finder
     * @param tag the tag of radius in an input file
     * @param radius0 vector in which coordinates are saved
     * @return true/false upon succes/failure
     */
    virtual bool readRadius(const std::string &tag, yarp::sig::Vector &radius0);
    
    /*******************************************************************/
    
    /** read the value on the diagonal line of a diagonal matrix and save them in the vector diag
     * @param rf a previously inizialized @see Resource Finder
     * @param tag the tag of the matrix in an input file
     * @param diag the vector in which values are saved
     * @param dimension the dimension of the matrix
     * @return true/false upon succes/failure
     */
    virtual bool readDiagonalMatrix(const std::string &tag,  yarp::sig::Vector &diag, const int &dimension);
    
    /*******************************************************************/
    
    public:
    
    /** class destructor
     */
    virtual ~Localizer(){};
    
    /*******************************************************************/
   
    /** configure parameters, input data and output files for the localization.
     * If some parameters are not provided by the user, they are set to default values ( see code)
     * @param rf a previously inizialized @see Resource Finder
     * @return true/false upon succes/failure
     */
    virtual bool configure(yarp::os::ResourceFinder &rf)=0;
    
    
    /*******************************************************************/
    
    /** returns the estimated pose x, the localization error and the execution
     * time
     * @param rf a previously inizialized @see Resource Finder
     * @return vector consisting of x, localization error and execution
     */
    virtual yarp::sig::Vector localization()=0;
    
    /*******************************************************************/
    
    
    /** save the result of the localization in an output file
     * @param rf a previously inizialized @see Resource Finder
     * @param ms_particle is a Vector of 8 components: the estimated 
     * x,y,z,phi,theta,psi, the final localization error and the execution time
     */
    virtual void saveData(const yarp::sig::Vector &ms_particle, const int &i)=0;
    
    
    
    
    /*******************************************************************/
    
    
    /** save statics data
     */
    virtual void saveStatisticsData(const yarp::sig::Matrix &solutions)=0;
    /*******************************************************************/

    
  
    /** send the result of the localization to a yarp port, that is opened and closed 
     * in this function
     * @param rf a previously inizialized @see Resource Finder
     * @param ms_particle is a Vector of 8 components: the estimated 
     * x,y,z,phi,theta,psi, the final localization error and the execution time
     */
    virtual void sendData(const  yarp::sig::Vector &ms_particle);
    
    
     /*******************************************************************/
    
    /** It is the execution of chosen localization algorithm
     */
    virtual void solve()=0;
   
    
   /*******************************************************************/
   
    /** initialize optimizer, get parameters
     *  initialize functions of the chosen localization algorithm
     */
    virtual void init()=0;
    
     /*******************************************************************/
     
    /** It is executed when the algorithm is finished.
     *  Compute final most significant particle, in world reference system.
     * @return most significant particle, as yarp Vector
     */ 
    virtual yarp::sig::Vector finalize()=0;
   
    /*******************************************************************/
    
    /** It is the execution of one iteration of ythe chosen algorithm
     * @return true when it is completed
     */
    virtual bool step()=0;
  
    
};

/*************************************************************************************************/
#endif

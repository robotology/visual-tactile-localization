/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef __ICUB_UTILS_H__
#define __ICUB_UTILS_H__

#include <iostream>
#include <string>
#include <fstream>
#include <yarp/sig/Image.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Event.h>
#include <yarp/os/Stamp.h>
#include <cv.h>
#include <highgui.h>

#include "GL/gl.h"
#if !defined(SIFTGPU_STATIC) && !defined(SIFTGPU_DLL_RUNTIME)
// SIFTGPU_STATIC comes from compiler
#define SIFTGPU_DLL_RUNTIME
// Load at runtime if the above macro defined
// comment the macro above to use static linking
#endif

#ifdef SIFTGPU_DLL_RUNTIME
   #include <dlfcn.h>
   #define FREE_MYLIB dlclose
   #define GET_MYPROC dlsym
#endif

#include "SiftGPU.h"

/**********************************************************/
class Utilities
{
public:
    SiftMatchGPU                            *matcher;
    SiftMatchGPU                            *matcherCheck;
    SiftGPU                                 *sift;
    std::vector<float >                     descriptors1, descriptors2;
    std::vector<SiftGPU::SiftKeypoint>      keys1, keys2;
    std::vector<cv::Point2f>                pointsL, pointsR;
    int num1, num2;
    bool writeS;
    void                                    *hsiftgpu;
    
    Utilities();
    ~Utilities();

    /**
     *  function to initialize the SIFT_GPU
     */
    void initSIFT_GPU();
     /**
     *   function to extract sifts using GPU
     */
    void extractMatch_GPU(cv::Mat &leftMat, cv::Mat &rightMat, double displacement=20.0);
    void extractMatch_GPU(cv::Mat &leftMat, cv::Mat &rightMat, cv::Mat &matMatches, double displacement=20.0);
    void getMatches(std::vector<cv::Point2f> & points1, std::vector<cv::Point2f>  &points2);
    void writeSIFTs(std::string filePath, std::vector<float> &des, std::vector<SiftGPU::SiftKeypoint>&points); 
    void writeMatch(std::string filePath,std::vector<cv::Point2f>  &pointsL, std::vector<cv::Point2f>  &pointsR);
};

#endif

//empty line to make gcc happy

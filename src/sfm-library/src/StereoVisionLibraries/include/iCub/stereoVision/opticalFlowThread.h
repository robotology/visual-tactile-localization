/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <opencv2/opencv.hpp>

#include <yarp/os/all.h>

#define DENSE    1

using namespace cv;

/**
* \ingroup StereoVisionLib
*
* The base class defining the 2D optical flow.
* It computes the 2D motion field in the image.
*/
class OpticalFlowThread : public yarp::os::PeriodicThread
{
private:

    cv::Mat optFlow;
    cv::Mat leftPrev;
    cv::Mat leftNext;
    bool done;
    bool work;
    bool dense;
    void computeFlowSparse(IplImage* previous, IplImage* current, Mat &optFlow);

public:

     OpticalFlowThread(yarp::os::ResourceFinder &rf);
    ~OpticalFlowThread() {};

    void setImages(cv::Mat &_leftPrev, cv::Mat &_leftNext);
    void getOptFlow(cv::Mat &_optFlow);
    void setFlow(int flowType);

    bool checkDone();

    bool threadInit();
    void threadRelease();
    void run();
    void onStop(void);
 
};

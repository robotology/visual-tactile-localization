#ifndef CAMERAPARAMETERS_H
#define CAMERAPARAMETERS_H

struct CameraParameters
{
public:
    int width;
    int height;

    double cx;
    double cy;
    double fx;
    double fy;

    bool initialized = false;
};

#endif /* CAMERAPARAMETERS_H */


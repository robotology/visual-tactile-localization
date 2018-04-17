# visual-tactile-localization

The code distributed here provides a reviewed implementation of the **Memory Unscented Particle Filter (MUPF)** algorithm for visual-tactile localization.

With respect to the original implementation the reviewed implementation offers:
- handling of variable number of measurements per time step;
- implementation of a translational motion model with velocity inputs;
- new [interface](headers/unscentedParticleFilter.h) that allows to
  - supply measurements to the filter as a `std::vector` of a variable number of
  `yarp::sig::Vector`
  - supply inputs (i.e. translational velocities) to the filter as a `yarp::sig::Vector`;
  - change the system noise covariance matrix during the filtering;
  - change the measurement noise during the filtering.

## Notes on implementation
This repository provides two main pieces of software:
- the filtering algorithm as a class (see [interface](headers/unscentedParticleFilter.h))
- a filtering module accepting data from `yarp` ports (see [code](src/localizer-module.cpp))

## Requirements
- [YARP](http://www.yarp.it/)
- [CGAL](https://www.cgal.org/)
- [libcgal-qt5-11]

## How to build (Linux)
Simply clone this repository to `$TAC_LOC` and then
```
mkdir $TAC_LAC/build
cd $TAC_LAC/build
cmake ../
make
```
The executable `upf-localizer` can be found in `$ROBOT-INSTALL/bin` where `$ROBOT-INSTALL` was provided while installing `yarp` using the option `-DCMAKE_INSTALL_PREFIX=$ROBOT_INSTALL`.

## How to run the the localizer
You can run the localizer as

```
upf-localizer --from $CONF_FILE_PATH
```
where `$CONF_FILE_PATH` is the path of a configuration file containing the following parameters:

```
[upf-module]
outputPath           <where to save output data>
estimateSourceFrame  <source frame for the Frame transform containing the estimate>
estimateTargetFrame  <target frame for the Frame transform containing the estimate>
robotSourceFrame     <source frame for the Frame transform from the inertial frame to the iCub root frame> (to be removed)
robotTargetFrame     <target frame for the Frame transform from the inertial frame to the iCub root frame> (to be removed)
pointCloudInputPort  <input port for visual informations>
contactsInputPort    <input port for tactile information>
period               <period>
visionQ              <system noise covariance matrix for visual filtering>
tactileQ             <system noise covariance matrix for tactile filtering>
visionR              <scalar measurement noise variance for visual measurements>
tactileR             <scalar measurement noise variance for tactile measurements>

[upf]
modelFile            <.OFF face-vertex mesh of the model of the object>
radius0              <3-tuple containing the sizes of the initial research region>
center0              <3-tuple containing the center of the initial research region>
N                    <number of particles to be used>
n                    <number of DoF>
R                    <default scalar measurement noise variance>
P0                   <initial state covariance>
Q0                   <default system noise covariance>
alpha                <Unscented Transform alpha parameter>
kappa                <Unscented Transform kappa parameter>
beta                 <Unscented Transform beta parameter>
nStepsBefRsmpl	     <number of steps to wait for before checking for Resampling>
nEffThr              <threshold for Particle Filter Neff parameter>
```

An example of configuration file may be found [here](config/config.ini).

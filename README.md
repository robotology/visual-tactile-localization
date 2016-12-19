# tactile-localization
The code distributed here provides the implementation of two different algorithms for the solution of the 6D object tactile localization: the Scaling Series algorithm, a reference algorithm in literature, and a new proposed algorithm, the Memory Unscented Particle Filter (MUPF). The MUPF is used in for the tactile object recognition, formulated as a localization problem with multiple models. The recognition solution is given by that object model who provides the minimum localization error.

## Prerequisities

Before compiling the code you are required to install 

1. [YARP](http://www.icub.org, and all the detailed information can be found at http://wiki.icub.org/wiki/Manual#Six._Software.2C_Compiling_YARP_and_iCub)
2. [CGAL](http://doc.cgal.org/latest/Surface_reconstruction_points_3/)

## How to compile
An example of compilation in Linux is given by:
```
mkdir build
cd build
ccmake ..
make install
```

## How to run 
You can run one of the algorithm typing in the command line:

```
localizer num_of_trials "mupf" --from configuration file
```

-`num_of_trial` is the number of times you want to run the algorithm and to have statistics about
-`"mupf"` string enables the use of MUPF algorithm. Otherwise, the scaling series is used.

## Publications

Memory Unscented Particle Filter for 6-DOF Tactile Localization,
G. Vezzani, U. Pattacini, G. Battistelli, L. Chisci, L. Natale, 
_submitted to IEEE Transaction on Robotics_, 2016,
preprint available on [arxiv:1607.02757v2](https://arxiv.org/abs/1607.02757v2)

A Novel Bayesian Filtering Approach to Tactile Object Recognition,
G. Vezzani, N. Jamali, U. Pattacini, G. Battistelli, L. Chisci, L. Natale, 
IEEE International Conference on Humanoid Robots, 2016, pp. 250 - 263


[![DOI:10.5281/zenodo.163860](https://zenodo.org/badge/20254/tacman-fp7/tactile-localization.svg)](https://zenodo.org/badge/latestdoi/20254/tacman-fp7/tactile-localization)


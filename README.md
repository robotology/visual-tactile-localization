#tactile-localization
The code distributed here provides the implementation of two different algorithms for the solution of the 6D object tactile localization: the Scaling Series algorithm, a reference algorithm in literature, and a new proposed algorithm, the Memory Unscented Particle Filter.

#dependeces 
Before compiling the code you are required to install 
1. YARP software: the website is http://www.icub.org, and all the detailed information can be found at http://wiki.icub.org/wiki/Manual#Six._Software.2C_Compiling_YARP_and_iCub;
2. CGAL libraries: the website is http://doc.cgal.org/latest/Surface_reconstruction_points_3/.

#compilation
The compilation makes use of the cmake tool.

#how to use the software
You can run one of the algorithm typing in the command line:

./localizer 'num_of_trials' 'mupf'or nothing --from configuration file

-num_of_trials is the number of times you want to run the algorithm and to have statistics about
-you have to write mupf if you want to use mupf algorithm. If you don't write anything, the scaling series is used

[![DOI](https://zenodo.org/badge/20254/tacman-fp7/tactile-localization.svg)](https://zenodo.org/badge/latestdoi/20254/tacman-fp7/tactile-localization)


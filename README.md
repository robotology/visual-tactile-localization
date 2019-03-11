# ⚙️ in-hand-object-tracking

The **in-hand-object-tracking** project is a _suite_ of applications for in-hand object tracking for the humanoid robot platform iCub.

The suite includes:
 - **object-tracking**: a visual-tactile in-hand object tracker combining partial point clouds and contact points within a 3D model-aided UPF
 - **object-tracking-viewer**: a visualizer that shows the object estimate, the ground truth and the point cloud of the scene
 - **object-tracking-ground-truth**: a marker-based ground-truth module for validation

# Overview
- [🎛 Dependencies](#-dependencies)
- [🔨 Build the suite](#-build-the-suite)


# 🎛 Dependencies
 in-hand-object-tracking suite depends on
 - [BayesFilters](https://github.com/robotology/bayes-filters-lib) - `version >= 0.9.100`
 - [iCub](https://github.com/robotology/icub-main)
 - [iCubContrib](https://github.com/robotology/icub-contrib-common)
 - [nanoflann](https://github.com/jlblancoc/nanoflann)
 - [OpenCV](http://opencv.org) - `version >= 3.3`
 - [OpenMP](https://www.openmp.org/) (optional)
 - [Open Asset Import Library, ASSIMP](http://assimp.org) - `version >= 3.0`
 - [SuperimposeMesh](https://github.com/robotology/superimpose-mesh-lib) - `version >= 0.10.100`
 - [VTK](https://vtk.org/) (optional)
 - [YARP](http://www.yarp.it)


# 🔨 Build the suite
Use the following commands to build, install and link the library.

### Build (object-tracking only)
`CMake` is used to build the suite:
```bash
$ git clone https://github.com/robotology-playground/visual-tactile-localization
$ cd visual-tactile-localization
$ mkdir build && cd build
$ cmake [-DUSE_OPENMP=ON] ..
$ make
$ [sudo] make install
```

The option `-DUSE_OPENMP=ON` is **optional**. If set to `ON`, the code is built using the library `OpenMP` for multithreaded execution. 

### Build the additional modules

#### Visualization module
The module `object-tracking-viewer` requires the library `VTK`.

In order to build the `object-tracking-viewer` module the following option is required when `cmake` is run:
```bash
$ cmake -DBUILD_OBJECT_TRACKING_VIEWER=ON ..
```
#### Ground truth module
The module `object-tracking-ground-truth` requires the library `OpenCV` to be built with the [extra modules](https://github.com/opencv/opencv_contrib) and the `ArUco` module activated (i.e. with the `cmake` option `BUILD_opencv_aruco` set to `ON`).

In order to build the `object-tracking-ground-truth` module the following option is required:
```bash
$ cmake -DBUILD_OBJECT_TRACKING_GROUND_TRUTH=ON ..
```

# ⚙️ in-hand-object-tracking

The **in-hand-object-tracking** project is a _suite_ of applications for in-hand object tracking for the humanoid robot platform iCub.

<p align="center"><img src="https://github.com/robotology-playground/visual-tactile-localization/blob/master/how_to_images/vtk_viewer_execution.png" alt="in hand object tracking" width="400" height="400"/></p>

The suite includes:
 - **object-tracking**: a visual-tactile in-hand object tracker combining partial point clouds and contact points within a 3D model-aided UPF
 - **object-tracking-viewer**: a visualizer that shows the object estimate, the ground truth and the point cloud of the scene
 - **object-tracking-ground-truth**: a marker-based ground-truth module for validation
 
# Try the code on your browser using GitPod
Run a tracking experiment directly on your browser using GitPod.

[![Gitpod](https://gitpod.io/button/open-in-gitpod.svg)](https://gitpod.io/#https://github.com/robotology/visual-tactile-localization)

See the section [Use the suite in a GitPod environment](#computer-use-the-suite-in-a-gitpod-environment) for instructions.


# Overview
- [🎛 Dependencies](#-dependencies)
- [:computer: Use the suite in a GitPod environment](#computer-use-the-suite-in-a-gitpod-environment)
- [🔨 Build the suite](#-build-the-suite)
- [:ok_hand: Run an experiment](#ok_hand-run-an-in-hand-object-tracking-experiment)


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
 
 **Important**: please use the `devel` branch for the libraries `BayesFilters` and `SuperimposeMesh`.
 
 **Tip:** if you don't have time to install all the dependecies you can use a GitPod environment directly on your browser, only few steps are required! See the next section for instructions.

# :computer: Use the suite in a GitPod environment
In order to use the suite in a GitPod environment, please follow these instructions:

1. Press on the following button [![Gitpod](https://gitpod.io/button/open-in-gitpod.svg)](https://gitpod.io/#https://github.com/robotology/visual-tactile-localization)

2. Register yourself on `GitPod` in case this is the first time you use it

After log-in, the GitPod system will prepare an environment with all the dependencies required by the suite - this may take some time. Once done, you will be prompted to a screen like this. 

<p align="center"><img src="https://user-images.githubusercontent.com/6014499/57372400-d8a0e680-7195-11e9-854f-74b658282143.png" alt="" width="320" height="240"/></p>

In order to access to the GUI of the environment:

3. Press on the **ports** button in the bottom-right corner of the screen

<p align="center"><img src="https://user-images.githubusercontent.com/6014499/57371365-d9844900-7192-11e9-8ce8-ebcc8be25d97.png" alt=""/></p>

4. Search for the port **6080** and press on the button **Open Browser**


<p align="center"><img src="https://user-images.githubusercontent.com/6014499/57371366-da1cdf80-7192-11e9-9abb-b937b4dbc945.png" alt=""/></p>

5. Press on the button **Connect**

<p align="center"><img src="https://user-images.githubusercontent.com/6014499/57373272-477f3f00-7198-11e9-92d5-30f48ba9894e.png" alt=""  width="320" height="240"/></p>

A Linux desktop environment will then be ready for you with all the required dependencies already installed.

<p align="center"><img src="https://user-images.githubusercontent.com/6014499/57371735-d6d62380-7193-11e9-8871-cd60253f0c15.png" alt="" width="320" height="240"/></p>

You can build the suite using the instructions from the [next](#-build-the-suite) section.


# 🔨 Build the suite
Use the following commands to build, install and link the library.

### Build (object-tracking only)
`CMake` is used to build the suite:
```bash
$ git clone https://github.com/robotology/visual-tactile-localization
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
# :ok_hand: Run an in-hand object tracking experiment
The tracking algorithm can be tested offline using a dataset provided in the following section.

Please follow these instructions.

## Instructions

### Download the dataset

> If you are using [GitPod](#computer-use-the-suite-in-a-gitpod-environment) it is not required to download the dataset. It is already available in the Desktop of the environment.

Download the [example dataset](https://figshare.com/articles/dataset_in_hand_tracking_iros_2019_zip/8029304) and unzip it. In the following the extracted folder will be identified as `$DATASET`.

#### Start the YARP server and manager

1. Open a new terminal and run the YARP server:
   ```
   $ yarpserver --write
   ```

2. On another terminal run the YARP manager:
   ```
   $ yarpmanager
   ```
   
   The YARP manager window will open.
   
#### Start the YARP dataplayer and load the data

1. Double click on the `Applications` entity in order to show the list of available applications.
    
    ![YARP manager applications](https://github.com/robotology-playground/visual-tactile-localization/blob/master/how_to_images/yarp_manager_applications.png)
    
2. Double click on the application named `Object_Tracking_on_iCub_(Play)`. A new tab will open on the right.

   ![YARP manager object tracking application](https://github.com/robotology-playground/visual-tactile-localization/blob/master/how_to_images/yarp_manager_obj_trk_application.png)
   
3. From the list of modules available within the application `Object_Tracking_on_iCub_(Play)` select the module named `yarpdataplayer` by clicking on it and open it by clicking on the green button as indicated in the following figure.

   ![YARP manager open dataplayer](https://github.com/robotology-playground/visual-tactile-localization/blob/master/how_to_images/yarp_manager_open_dataplayer.png)
   
4. The YARP dataplayer will open. This module is required to playback the data stored within the folder `$DATASET`.

5. Open the dataset by clicking on `File`, then on `Open Directory` in the menu available in the top of the window.

   ![YARP dataplayer open directory](https://github.com/robotology-playground/visual-tactile-localization/blob/master/how_to_images/dataplayer_open_directory.png)
   
6. A browse dialog will open. Select the folder `$DATASET` and click on the button `Choose`. If the data is loaded correctly, the dataplayer window should look like the following.

   ![YARP dataplayer open success](https://github.com/robotology-playground/visual-tactile-localization/blob/master/how_to_images/dataplayer_open_success.png)
   
   Now the data is ready to be played back! :thumbsup:
   
#### Start the other modules
1.  Go back to the YARP manager window and open all the remaining modules by clicking on the green button as indicated in the following figure.
 
   ![YARP run remaining modules](https://github.com/robotology-playground/visual-tactile-localization/blob/master/how_to_images/yarpmanager_run_remaining_modules.png)
   
2. In few seconds, you should see a green tick on the left of each module name as in the following figure.

    ![YARP all modules run](https://github.com/robotology-playground/visual-tactile-localization/blob/master/how_to_images/yarp_manager_all_applications_success.png)
    
    Several windows will appear:
    - two instances of a YARP viewer, e.g. an image viewer: one shows the images from the left eye of the robot iCub; the other shows the same image with the current bounding box of the object and the convex hull enclosing the robot hand superimposed. Please note that initially the viewers will be black since the experiment is not running yet.
    
    ![YARP view blank](https://github.com/robotology-playground/visual-tactile-localization/blob/master/how_to_images/yarpview_empty.png)
    
    - an instance of a 3D viewer showing a 3D reconstruction of the scene and comprising:
      - the point cloud of scene
      - the estimate of the object (in gray)
      - the ground truth (in transparent green)
      - the current pose of the hand of the robot

    Please note that initially the viewer will be uninitialized, as shown in the following figure, since the experiment is not running yet.
     
     <img src="https://github.com/robotology-playground/visual-tactile-localization/blob/master/how_to_images/vtk_viewer_empty.png" alt="VTK viewer blank" width="400" height="400"/>
     
     Now all the required modules are running! :tada: 
     
#### Connect the modules
1. Before starting the experiment, it is required to **connect** all the modules by clicking on the green button as indicated in the following figure.

    ![YARP manager connect all](https://github.com/robotology-playground/visual-tactile-localization/blob/master/how_to_images/yarp_manager_connect_all.png)
    
2. Look at the bottom of the YARP manager window. All the connections should be reported in green as **connected** under the column `Status`. If any of the connections is displayed with a red colour as **disconnected**, please click again on the green button as per step `11` until all the connections are green.

    ![YARP manager connections succesful](https://github.com/robotology-playground/visual-tactile-localization/blob/master/how_to_images/yarp_manager_all_connections_success.png)
    
#### Run the filtering algorithm
In order to run the filtering algorithm open a terminal and connect to the filtering module:
```
$ yarp rpc /object-tracking/cmd:i
```
Then start the filtering recursion by typing:
```
>> run_filter
```
In case of success, the following response is displayed:
```
Response: [ok]
```

#### Start the experiment
To start the experiment press the `Play` button on the `yarpdataplayer` window as shown in the following figure.

Please note that if you are running the experiment on [GitPod](#computer-use-the-suite-in-a-gitpod-environment), you may experience some performance degradation due to the limited computing capabilities of the environment.
    
![YARP dataplayer play](https://github.com/robotology-playground/visual-tactile-localization/blob/master/how_to_images/dataplayer_execution.png)
    
Once the experiment is started, move to the 3D viewer window and press the key `R` in order to reset the view. In order to zoom use the mouse scroll wheel. In order to move the point of view, press and keep pressed the left button of the mouse and move until the desired view is obtained. An example 3D view is shown in the following figure.

<img src="https://github.com/robotology-playground/visual-tactile-localization/blob/master/how_to_images/vtk_viewer_execution.png" alt="VTK viewer blank" width="400" height="400"/>
    
The YARP viewer window shows the current bounding box enclosing the object, in green, and the convex hull enclosing the robot hand, in red.
    
![YARP view execution](https://github.com/robotology-playground/visual-tactile-localization/blob/master/how_to_images/yarp_view_execution.png)
    
In order to restart the experiment, first reset the filtering module by typing:
 ```
 >> reset_filter
 ```
Then, press again the `Play` button on the `yarpdataplayer` window.

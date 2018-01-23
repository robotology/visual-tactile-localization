# visual-tactile-localization with motion scenario

![Example of localization experiment](misc/localization.gif)

The code distributed here provides a reviewed implementation of the **Unscented Particle Filter (UPF)** algorithm for visual-tactile localization.

(The previous __static only__ implementation can be found [here](https://github.com/robotology-playground/tactile-localization/releases/tag/v3.0)).

With respect to the original implementation the reviewed implementation offers:
- handling of multiple contact points per time step;
- implementation of an ideal measurement equation for testing purposes;
- implementation of a translational motion model with velocity inputs;
- new [interface](headers/unscentedParticleFilter.h) that allows to
  - supply measurements to the filter as a `std::vector` of a variable number of
  CGAL::Simple_cartesian<double>::Point_3 Point points.
  - supply inputs (i.e. translational velocities) to the filter as a `yarp::sig::Vector`;
  - change the artificial system noise covariance matrix during the filtering;

## Requirements
- [YARP](http://www.yarp.it/)
- [CGAL](https://www.cgal.org/)
- [libcgal-qt5-11]
- [VCG](http://vcg.isti.cnr.it/vcglib/) ([provided](headers/vcglib) within this repository)

## How to build (Linux)
Simply clone this repository to `$TAC_LOC` and then
```
mkdir $TAC_LAC/build
cd $TAC_LAC/build
cmake ../
make
```
The executable `localizer` can be found in `$TAC_LOC/build/bin`.

## How to run the offline localizer with a _motion scenario_
You can run an __offline localizer__ simulating a _motion scenario_ as
```
cd $TAC_LOC/build/bin
./localizer --from $CONF_FILE_PATH
```
where `$CONF_FILE_PATH` is the path of a configuration file containing the following parameters:
- `numTrials`, number of trials to be performed;
- `modelFile`, path of the [`.OFF`](https://en.wikipedia.org/wiki/OFF_(file_format)) containing the triangular mesh of the object;
- `auxCloud1File`, path of the [`.OFF`](https://en.wikipedia.org/wiki/OFF_(file_format)) containing very few contact points used during a pushing phase;
- `auxCloud2File`, path of the [`.OFF`](https://en.wikipedia.org/wiki/OFF_(file_format)) containing very few contact points used during an additional pushing phase;
- `modelOutputPath`, where to save the `.OFF` file containing the model of the object in the real and estimated pose, one file is saved for each filtering step in `modelOutputPath/trial<i>/` where `i` is the index of the current trial;
- `measOutputPath`, where to save the `.OFF` file containing the measurements used in each filtering step, one file is saved for each filtering step in `measOutputPath/trial<i>/` where `i` is the index of the current trial;
- `resultsOutputPath` where to save a `.CSV` file containing a summary about all the filtering steps (real pose, estimated pose, position and velocity of the reference point, yaw rate, origin of the observer and execution time for each step). The file is saved in `resultsOutputPath/trial<i>` where `i` is the index of the current trial;
- `radius0`, radius of the initial research region as a list of three doubles;
- `center0`, center of the initial research region as a list of three doubles;
- `N`, number of particles;
- `n`, number of DoFs;
- `R`, scalar measurement noise variance;
- `P0`, 6-dimensional diagonal covariance matrix of the initial state as a list of six doubles;
- `Q0`, 6-dimensional diagonal covariance matrix of the initial system noise vector;
- `alpha`, Unscented Transform parameter;
- `kappa`, Unscented Transform parameter;
- `beta`, Unscented Transform parameter;
- `nEffThr`, threshold that is compared to Neff to decide whether to perform resampling or not;
- `nStepsBefRsmpl`, number of steps to wait for before checking for Neff and possibly perform resampling;
- `useIdealMeasEqn`, whether to use the ideal measurement equation or not (true/false);
- `observerOrigin`, a 3D tuple containing the origin of the observer and used to generate a more realistic point cloud;
- `useCenterVelocity`, whether to use or not the velocity of the center of the object as input to the filter (see the section [Integration](#integration) for more details on this setting);
- `numContacts`, fixed number of contact points to be processed at each time step during __static__ motion phases (see the section [Integration](#integration) for more details on this setting).

## Notes on an example motion scenario
An example configuration file is [available](configurationFiles/configMustard_sim_motion.ini).

The model used in the simulated scenario is that of a mustard bottle (taken from the [YCB dataset](http://www.ycbbenchmarks.com/object-models/)). The mustard bootle moves according to the following plan
- it stands still initially (measurements are simulated by Disk Poisson sampling the model `modelFile` - in this phase localization "using" vision is simulated);
- then a first pushing movement is simulated (the auxiliar point cloud `auxCloud1File` contains three contact points expressed in object fixed frame used as measurements during the pushing phase - in this phase localization "using" touch is simulated);
- it stops (measurements are simulated again by Disk Poisson sampling the model `modelFile` - in this phase localization "using" vision is simulated);
- a second pushing movement is simulated (using the auxiliar point cloud `auxCloud2File` - in this phase localization "using" touch is simulated);
- it stops again (measurements are simulated again by Disk Poisson sampling the model `modelFile`- in this phase localization "using" vision is simulated);

The plan expressed above can be easily changed in the file [main.cpp](src/main.cpp) by defining suitable `LocalizationPhase` phases and setting them within the localizer using the method `LocalizerMotion::setLocPhase`. More details on the `LocalizationPhase` can be found in the next section.

## Notes on implementation

### Offline localizer
The [localizer](headers/localizer-motion.h) is implemented as a `yarp::os::RFModule` that uses the UPF filter via this [interface](headers/unscentedParticleFilter.h), a fake point cloud generator via this [interface](headers/fakePointCloud.h) and a trajectory generator via this [interface](headers/motionGenerator.h). The method `main` within the file [main.cpp](src/main.cpp) instantiates the point cloud and trajectory generators and the RFModule.

### UPF Filter
The filter can be used through the [interface](headers/unscentedParticleFilter.h).

The main methods offered are

```
bool configure(yarp::os::ResourceFinder &rf);
```
that takes a previously instantiated `yarp::os::ResourceFinder` and configure the filter.

```
void init();
```
that reset the filter.

```
void setNewMeasure(const Measure &m);
```
that set a new measure.

```
void setNewInput(const yarp::sig::Vector &in);
```
that set a new input.

```
void setQ(const yarp::sig::Vector &covariance);
```
that set the current artificial system noise covariance matrix.

```
void setRealPose(const yarp::sig::Vector &pose);
```
that set the real pose used to evaluate the ideal measurement equation.

```
void step();
```
that perform a filtering step.

```
yarp::sig::Vector getEstimate();
```
that extract the MAP estimate.
```
double evalPerformanceIndex(const yarp::sig::Vector &estimate, const std::deque<Point> &points);
```
that evaluates the performance index taking into account the estimate given, the contact points given and
the model loaded through the configuration file.
```
void transformObject(const yarp::sig::Vector &estimate, Polyhedron &transformed);  
```
that transform the model of the object, with coordinates expressed in a object fixed reference frame, into a model
of the object in the estimated pose given with coordinates expressed in robot reference frame.

### Fake Point Cloud
Fake point clouds are generated using the class `FakePointCloud` available through this [interface](headers/fakePointCloud.h).

The main methods offered are
```
bool loadObjectModel(const std::string &file_path);
```
that loads a [`.OFF`](https://en.wikipedia.org/wiki/OFF_(file_format)) file. It may contains both a __Face-Vertex triangular mesh__ or a __Vertex only__ description representing a point cloud. Coordinates must be expressed in an __object fixed__ reference frame.

```
void setPose(const yarp::sig::Vector &pose);
```
that set the current pose of the object. The pose is expressed as 6-dimensional vector containing the position of the object center w.r.t the robot reference frame and three Euler ZYZ angles representing the attitude w.r.t the robot reference frame.

```
void transformModel(simpleTriMesh &mesh_out);
```
that transform the model (both mesh or point cloud depending on the type of file loaded with `FakePointCloud::loadObjectModel`) using the current pose set with `FakePointCloud::setPose`.

 ```    
 void samplePointCloud(std::vector<Point> &cloud,const yarp::sig::Vector &obs_origin, const int &num_points);
 ```
that create a point cloud by Disk Poisson sampling the whole surface of the object in the current pose set with `FakePointCloud::setPose` and taking the points visible to the observer whose origin is at `obs_origin`. It requires a model of the object containing a triangular mesh and not a point cloud (i.e. a Vertex only model). The method tries to sample exactly `num_points` points.

```
void getPointCloud(std::vector<Point> &cloud);
```
that return the original point cloud loaded with `FakePointCloud::loadObjectModel` after a rototranslation according to the curent pose set with `FakePointCloud::setPose`.

### Trajectory generator
Trajectories are generated using the classes `StaticMotionGenerator` and `PolynomialMotionGenerator` available through this [interface](headers/motionGenerator.h).

#### Abstract class `MotionGenerator`
An abstract class `MotionGenerator` offers the following methods

```
void setPeriod(const double &dt);
```
that set the sampling period of the trajectory.

```
void setDuration(const double &T);
```
that set the total duration of the trajectory.

```
void setInitialRefPosition(const yarp::sig::Vector &pos);
```
that set the initial position of a __reference__ point, e.g. a point of interest on the surface of the object.
This point can be the center of the object or any other point and this depends on a __displacement vector__ that decides
where the center of the object is located w.r.t the reference point. The displacement can be decided using the method `MotionGenerator::setDisplToCenter`.

```     
void setDisplToCenter(const yarp::sig::Vector &displ);
``` 
that set the displacement vector from the __reference__ point to the center of the object. It should be noted that the center of the object is the point belonging to volume of the object having coordinates (0, 0, 0) in a object fixed reference frame. In order to have a reference point corresponding to the center of the object it suffices to set `displ` to zero.

```
void setInitialYaw(const double &yaw);
```
that set the initial yaw attitude of the object. (Only yaw attitude is implemented so far).

```
virtual bool step();
```
that perform a step in the trajectory. It should be called before ```MotionGenerator::getMotion```. This function can be reimplemented if required.

```
void reset();
```
that reset the internal time of the trajectory to a value equal to `-1 * sampling_period` with `sampling_period` the sampling period set with `MotionGenerator::setPeriod`. This way the first time `MotionGenerator::step` is called the method `MotionGenerator::getMotion` returns the state of the trajectory at time `t=0`.

```     
virtual void getMotion(yarp::sig::Vector &pos, yarp::sig::Vector &ref_pos, yarp::sig::Vector &vel, yarp::sig::Vector &ref_vel, double &yaw, double &yaw_rate) = 0;
```
that is pure virtual method and must be implemented. It returns the current position `pos` of the center of the object, the current position `ref_pos` of the reference point, the current linear velocity `vel` of the center of the object (obtained composing the velocity of the reference point and the angular velocity of the object), the current linear velocity `ref_vel` of the reference point, the current yaw attitude `yaw` of the object and the current yaw rate `yaw_rate` of the object.

#### Derived class `StaticMotionGenerator`
A derived class from `MotionGenerator` is `StaticMotionGenerator`. 

The method `StaticMotionGenerator::getMotion` always returns the initial position and attitude and zero velocity.

#### Derived class `PolynomialMotionGenerator`
Another derived class is `PolynomialMotionGenerator` that generates 5-th order trajectories for each cartesian coordinate of the reference point and for the yaw angle. The velocity of the center of the object is obtained taking into account the displacment from the reference point to the center and the instantaneous angular velocity. The position of the center of the object is obtained by forward Euler integrating the velocity of the center of the object. All the trajectories are with zero initial and final velocity and acceleration.

It offers the following methods
```
void setFinalRefPosition(const yarp::sig::Vector &pos);
```
that set the final position of the reference point.

```
void setFinalYaw(const double &yaw);
```
that set the final yaw angle of the object.

```
void initTrajectory();
```
that initialize the trajectory taking into account the duration of the trajectory and the initial and final conditions.

```
void getMotion(yarp::sig::Vector &pos, yarp::sig::Vector &ref_pos, yarp::sig::Vector &vel, yarp::sig::Vector &ref_vel, double &yaw, double &yaw_rate) override;    

```
that overrides the pure virtual method of `MotionGenerator`.

### Integration
All the classes presented above are combined together in the class `LocalizerMotion`.

In order to specify multi-phase trajectories the class `LocalizerMotion` expects that all the motion phases are specified using the method `LocalizerMotion::setLocPhase`. In this implementation this happens in the method `main` in the file [main.cpp](src/main.cpp).

#### Struct `LocalizationPhase`
Each phase is represented using a `struct` that contains the following fields:
- `type`, the type of phase, it can be `LocalizationType::Static` or `LocalizationType::Motion`;
- `displ`, the displacement from the reference point to the center of the object to be used during this motion phase;
- `holdDisplFromPrevious`, that decides whether to inherit the displacement (between the reference point and the center of the object) from the previous phase.
- `ref_pos_0`, the initial position of the reference point;
- `yaw_0`, the initial yaw angle;
- `delta_pos`, the relative displacement between the initial and final position of the referene point;
- `delta_yaw`, the relative change between the initial and final yaw angle;
- `step_time`, the sampling period of the trajectory associated to the phase;
- `num_points`, the number of points of the point clouds used in an iteration of a `LocalizationType::Static` phase;
- `duration`, the total duration of the trajectory associated to the phase;
> for `LocalizationType::Static` phases the `step_time` and `duration` fields should be used to decide how many iterations of static localization have to be performed. To have `n` iterations it suffices to set `step_time` to `1` and `duration` to `n`.
- `pc`, pointer to a `FakePointCloud`;
> For `LocalizationType::Static` phases it must be a `FakePointCloud` loaded with a __Vertex-Face triangular mesh__ of the object because during a static phase measurements are obtained by Disk Poisson sampling the surface of the object. For each iteration of the static localization a point cloud with `num_points` points is generated and each filtering step is performed with a measurement vector containing `numContacts` contact points where `numContacts` is specified in the configuration file.

> For `LocalizationType::Motion` phases it must be a `FakePointCloud` loaded with a __Vertex only__ model of the object containing, e.g., the location of few contact points used in an hypothetic __pushing__ phase.

- `mg`, pointer to a `MotionGenerator`.
> For `LocalizationType::Static` phases it should be a `StaticMotionGenerator`.

> For `LocalizationType::Motion` phases it should be a `PolynomialMotionGenerator`.

#### Configuration of a localization phase
The constructor of the struct `LocalizationPhase`
```
LocalizationPhase(const LocalizationType type, const double &duration,const double &step_time, MotionGenerator *mg, FakePointCloud *pc, const bool holdDispl = false)    
```
requires the type of the phase, the duration, the step time, the motion generator and the fake point cloud. By default the displacement between the reference point and the center of the object is not inherited from the previous phase.

As regards the other fields described above some of them are filled automatically in the method `LocalizerMotion::updateModule` that configures each localization phase before performing the first filtering step associated to that phase.
In particular 
- the initial conditions of a phase are set equal to the final conditions of the previous phase (this behavior is not true for the first phase added to the localizer);
- if a motion phase is `LocalizationType::Static` then the displacements `delta_pos` and `delta_yaw` are automatically set to 0;
- if a motion phase is configured with the `holdDisplFromPrevious` set to `true` then the displacement is copied from the previous phase;
- if a motion phase is configured with the `holdDisplFromPrevious` set to `false` a new displacement vector __have__ to be specified for that phase and the initial position of the __new__ reference point, expressed in robot reference frame, is automatically calculated taking into account the final yaw attitude of the previous phase.

#### Velocity of the center vs. velocity of the reference point
Please note that depending on the option `useCenterVelocity` specified in the configuration file the filter will use the velocity of the center of the object (`useCenterVelocity = true`) or the velocity of the reference point (`useCenterVelocity = false`).

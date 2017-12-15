# visual-tactile-localization

The code distributed here provides a reviewed implementation of the **Memory Unscented Particle Filter (MUPF)** algorithm for tactile localization.

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
- `modelFile`, path of the [`.OFF`](https://en.wikipedia.org/wiki/OFF_(file_format)) containing the triangular mesh of the object;
- `auxCloud1File`, path of the [`.OFF`](https://en.wikipedia.org/wiki/OFF_(file_format)) containing very few contact points used during a pushing phase;
- `auxCloud2File`, path of the [`.OFF`](https://en.wikipedia.org/wiki/OFF_(file_format)) containing very few contact points used during an additional pushing phase;
- `modelOutputPath`, where to save the `.OFF` file containing the model of the object in the real and estimated pose, one file is saved for each filtering step;
- `measOutputPath`, where to save the `.OFF` file containing the measurements used in each filtering step, one file is saved for each filtering step;
- `resultsOutputPath` where to save a `.CSV` file containing a summary about all the filtering steps (real pose, estimated pose for each step);
- `radius0`, radius of the initial research region as a list of three doubles;
- `center0`, center of the initial research region as a list of three doubles;
- `N`, number of particles;
- `n`, number of DoFs;
- `R`, scalar measurement noise variance;
- `P0`, 6-dimensional covariance matrix of the initial state as a list of six doubls;
- `alpha`, Unscented Transform parameter;
- `kappa`, Unscented Transform parameter;
- `beta`, Unscented Transform parameter;
- `memoryWidth`, width of the memory window of the Memory UPF;
- `resampleInFirstIters`, whether to resample also in the first two iterations of the algorithm (true/false);
- `useIdealMeasEqn`, whether to use the ideal measurement equation or not (true/false);
- `numContacts`, fixed number of contact points to be processed at each time step.

## Notes on an example motion scenario
An example configuration file is [available](https://github.com/robotology-playground/tactile-localization/blob/feature/motion_model/configurationFiles/configMustard_sim_motion.ini).

The model used in the simulated scenario is that of a mustard bottle (taken from the [YCB dataset](http://www.ycbbenchmarks.com/object-models/)). The mustard bootle moves according to the following plan
- it stands still initially (measurements are simulated by Disk Poisson sampling the model `modelFile`);
- then a first pushing movement is simulated (the auxiliar point cloud `auxCloud1File` contains three contact points expressed in object fixed frame used as measurements during the pushing phase);
- it stops (measurements are simulated again by Disk Poisson sampling the model `modelFile`);
- a second pushing movement is simulated (using the auxiliar point cloud `auxCloud2File`);
- it stops again (measurements are simulated again by Disk Poisson sampling the model `modelFile`);

More details on the implementation of the multi-phase trajectory generator can be found in the method `updateModule` of the class `LocalizerMotion` ([sources](src/localizer-motion.cpp)). The phases of the motion can not be chosen using the configuration file and must be decided within the method `updateModule`.

## Notes on implementation

## Offline localizer
The [localizer](headers/localizer-motion.h) is implemented as a `yarp::os::RFModule` that uses the [interface](headers/unscentedParticleFilter.h) of the MUPF. The [main](src/main.cpp) instantiates the RFModule.

## MUPF Filter
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

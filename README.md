# visual-tactile-localization

The code distributed here provides a reviewed implementation of the **Memory Unscented Particle Filter (MUPF)** algorithm for tactile localization.

With respect to the original implementation the reviewed implementation offers:
- handling of multiple contact points per time step;
- implementation of an ideal measurement equation for testing purposes;
- new [interface]() to supply measurements to the filter as a `std::vector` of a variable number of
  CGAL::Simple_cartesian<double>::Point_3 Point points.                                                                       

## Requirements
- [YARP](http://www.yarp.it/)
- [CGAL](https://www.cgal.org/)

## How to build (Linux)
Simply clone this repository to `$TAC_LOC` and then
```
mkdir $TAC_LAC/build
cd $TAC_LAC/build
cmake ../
make
```
The executable `localizer` can be found in `$TAC_LOC/build/bin`.

## How to run the offline localizer
You can run an __offline localizer__ taking measurements from a file as
```
cd $TAC_LOC/build/bin
./localizer --from $CONF_FILE_PATH
```
where `$CONF_FILE_PATH` is the path of a configuration file containing the following parameters:
- `modelFile`, path of the [`.OFF`](https://en.wikipedia.org/wiki/OFF_(file_format)) containing the triangular mesh of the object;
- `measurementsFile`, path of the [`.OFF`](https://en.wikipedia.org/wiki/OFF_(file_format)) containing the contact points (measurements) as the vertices of the `.OFF` file;
- `modelOutputPath`, where to save the `.OFF` file containing the model of the object in the estimated pose, one file for each trial;
- `trialsOutputPath`, where to save a `.CSV` file containing a summary about all the trials (execution time, performance index, solutions found, etc);
- `radius0`, radius of the initial research region as a list of three doubles;
- `center0`, center of the initial research region as a list of three doubles;
- `N`, number of particles;
- `n`, number of DoFs;
- `Q`, 6-dimensional system noise covariance matrix as a list of six doubles;
- `R`, scalar measurement noise variance;
- `P0`, 6-dimensional covariance matrix of the initial state as a list of six doubls;
- `alpha`, Unscented Transform parameter;
- `kappa`, Unscented Transform parameter;
- `beta`, Unscented Transform parameter;
- `memoryWidth`, width of the memory window of the Memory UPF;
- `resampleInFirstIters`, whether to resample also in the first two iterations of the algorithm (true/false);
- `useIdealMeasEqn`, whether to use the ideal measurement equation or not (true/false);
- `numTrials`, number of trials to be executed as an integer;
- `numContacts`, fixed number of contact points to be processed at each time step;
- `realPose`, real pose of the object used by the ideal measurement equation.

## Notes on implementation

## Offline localizer
The [localizer](headers/localizer.h) is implemented as a `yarp::os::RFModule` that uses the [interface](headers/unscentedParticleFilter.h) of the MUPF. The [main](src/main.cpp) instantiates the RFModule.

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

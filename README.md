# Unscented Kalman Filter

<img src="media/ukf_highway_tracked.gif" width="700" height="400" />

This project implements an Unscented Kalman Filter (UKF) to estimate the state of multiple cars on a highway using noisy lidar and radar measurements.
Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric. 

<img src="media/ukf_highway.png" width="700" height="400" />

The `main.cpp` uses `highway.h` to create a straight 3 lane highway environment with 3 traffic cars and the main ego car at the center. 
The viewer scene is centered around the ego car and the coordinate system is relative to the ego car as well.
The ego car is green while the other traffic cars are blue.
The traffic cars will be accelerating and altering their steering to change lanes.
Each of the traffic car's has it's own UKF object generated for it, and will update each individual one during every time step. 
The red spheres above cars represent the (x,y) lidar detection and the purple lines show the radar measurements with the velocity magnitude along the detected angle.
The Z axis is not taken into account for tracking, so it is only tracking along the X/Y axis.

## Dependencies

* `cmake >= 3.5`
  * For all OSes click [here][CMake] for installation instructions
* `make >= 4.1`
  * For Linux `make` is installed by default on most distros.
  * For macOS install [Xcode] command line tools to get make.
  * For Windows click [here][Make for Windows] for installation instructions.
* `gcc/g++ >= 5.4`
  * For Linux  `gcc` and `g++` are installed by default on most distros.
  * For macOS same deal as `make`, install [Xcode] command line tools.
  * For Windows is recommended using [MinGW].
* `pcl >= 1.2`

## Basic build instructions

1. Clone this repo.

**SSH clone***

```bash
git clone git@github.com:alvgaona/unscented-kalman-filter.git
```

**HTTPS clone**

```bash
git clone https://github.com/alvgaona/unscented-kalman-filter.git
```

2. Execute make command

```bash
make build
```

4. Run it.
 
 ```bash
cd build
./ukf
 ```

## Code Style

This project is based on [Google's C++ style guide] plus a custom options found in `[.clang-format]`.

## Documents

The documents can be found under [docs][Documentation].

- [Data generation][Data Generation]
- [Rubric points][Rubric Points]

[CMake]: https://cmake.org/install
[Xcode]: https://developer.apple.com/xcode/features
[Make for Windows]: http://gnuwin32.sourceforge.net/packages/make.htm
[MinGW]: http://www.mingw.org
[Google's C++ style guide]: https://google.github.io/styleguide/cppguide.html
[Documentation]: docs/
[Data Generation]: docs/GeneratingData.md
[Rubric Points]: docs/RubricPoints.md
[.clang-format]: .clang-format

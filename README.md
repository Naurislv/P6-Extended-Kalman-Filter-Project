# Extended Kalman Filter Project
#### Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)

[image1]: ./data_1_Laser.png "data_1_Laser"
[image2]: ./data_2_Laser.png "data_2_Laser"
[image3]: ./data_1_Radar.png "data_1_Radar"
[image4]: ./data_2_Radar.png "data_2_Radar"
[image5]: ./data_1_Fusion.png "data_1_Fusion"
[image6]: ./data_2_Fusion.png "data_2_Fusion"
[image7]: ./KalmanFilter_LidarRadar_map.png "data_2_Fusion"

---
## Summary

This is Udacity Self-Driving Car Engineer Nanodegree program Term 2 first project. Goal is to code Extended Kalman filter in C++. This repo includes also python implementation of Kalman Filter (2D and not Extended) - can be found in python directory.

* Eigen - C++ library for operations with matrices
* KalmanFilter1D_cpp - Kalman Filter 1D iplementation in C++
* data - input Lidar/Data tracking sources data
* python - python implementation of KalmanFilter

To run code search for 'Basic build instructions'

Extended Kalman filter road map:
![alt text][image7]

Starter code was forked (This is not actual Fork): https://github.com/udacity/CarND-Extended-Kalman-Filter-Project

More tools to generate and plot graphs can be found: https://github.com/udacity/CarND-Mercedes-SF-Utilities

## Dependencies

* cmake >= 3.5
* All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
 * Linux: make is installed by default on most Linux distros
 * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
 * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
 * Linux: gcc / g++ is installed by default on most Linux distros
 * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
 * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic build instructions

1. Clone this repo.
2. Make a build directory: ```mkdir build && cd build```
3. Compile: ```cmake .. && make```
    * On windows, you may need to run: cmake .. -G "Unix Makefiles" && make
4. Run it: ```./ExtendedKF path/to/input.txt path/to/output.txt```. You can find some sample inputs in 'data/'.
    * eg. ```./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt```

## Plot results

1. Follow 'Basic build instructions' to generate output.txt
2. Open visualize_results.ipynb
3. Run each block

### Useful Resources

* The Kalman Filter video lectures: https://www.youtube.com/playlist?list=PLX2gX-ftPVXU3oUFNATxGXY90AULiqnWT
* C++ cheatsheet can be found in repository: Cpp_cheatsheet
* KalmanFilter_cheatsheet PDF (related to lessons) : KalmanFilter_cheatsheet
* Interactive Book on github: https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python
* Robert F. Stengel: Optimal Control and Estimation (Dover Books on Mathematics):  https://goo.gl/xibjAK
* Welch, G. and Bishop, G., “An Introduction to the Kalman Filter”,  SIGGRAPH 2001: https://goo.gl/GhdrPi
* How Kalman filter works, in pictures: http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/

### Results

| RMSE          | px     | py     | vx     | vy     |
|---------------|--------|--------|--------|--------|
| Data 1 RADAR  | 0.1012 | 0.0823 | 0.6134 | 0.5819 |
| Data 2 RADAR  | 0.1511 | 0.2044 | 0.1044 | 0.1288 |
| Data 1 LASER  | 0.0682 | 0.0572 | 0.6256 | 0.5609 |
| Data 2 LASER  | 0.2180 | 0.1943 | 0.9375 | 0.8339 |
| Data 1 Fusion | 0.0651 | 0.0605 | 0.5432 | 0.5442 |
| Data 2 Fusion | 0.1832 | 0.1903 | 0.4778 | 0.8056 |

![alt text][image1]
![alt text][image2]
![alt text][image3]
![alt text][image4]
![alt text][image5]
![alt text][image6]

[![Pull requests](https://img.shields.io/github/issues-pr-raw/xyyeh/robotics_exp_proj.svg)](https://github.com/xyyeh/robotics_exp_proj/pulls)
[![Opened issues](https://img.shields.io/github/issues-raw/xyyeh/robotics_exp_proj.svg)](https://github.com/xyyeh/robotics_exp_proj/issues)
[![Documentation](https://img.shields.io/badge/Documentation-latest-blue.svg)](https://github.com/xyyeh/robotics_exp_proj/)

# Rigid Body Dynamics Library
The library wraps the library from [RBDyn](https://github.com/jrl-umi3218/RBDyn) and provides a reduced set of frequently used functions to compute dynamic parameters of branching rigid body robotic systems. This can be easily integrated into a real-time controller to implement task/primitive level control and planning strategies.

Installation
------
For first time installation, users can install and test out the functionalities in a dedicated folder before integrating it with other libraries. This can be done by specifying a <develoment folder> for CMAKE_INSTALL_PREFIX. To automatically setup third party libraries, simply set the BUILD_THIRD_PARTY flag
```bash
mkdir build
cmake .. -DCMAKE_INSTALL_PREFIX=<development folder> -DBUILD_THIRD_PARTY=ON
```
For subsequent builds, BUILD_THIRD_PARTY can be dropped.

### Dependencies
 * [Git]()
 * [CMake]() >= 3.1
 * [SpaceVecAlg](https://github.com/jrl-umi3218/SpaceVecAlg) v.1.1.0
 * [RBDyn](https://github.com/jrl-umi3218/RBDyn) v.1.2.1

Documentation
-----
Features:
 * Forward kinematics to compute end effector pose, jacobian and time-derivative of jacobian
 * Computation of matrices commonly used in inverse dynamics controllers, i.e. mass matrix, centrifugal/coriolis and gravity loading vectors.
 * Computation of coriolis matrix used for momentum-based collision detection algorithms
 
 

iDynTree [![Build Status](https://travis-ci.org/robotology/idyntree.svg?branch=maste2013-r)](https://travis-ci.org/robotology/idyntree) [![Build status](https://ci.appveyor.com/api/projects/status/1uecfmyvxb2dujt9/branch/master?svg=true)](https://ci.appveyor.com/project/robotology/idyntree/branch/master) [![License: LGPL v3](https://img.shields.io/badge/License-LGPL%20v3-blue.svg)](https://www.gnu.org/licenses/lgpl-3.0) [![License: LGPL v2](https://img.shields.io/badge/License-LGPL%20v2-blue.svg)](https://www.gnu.org/licenses/lgpl-2.1) [![CLA assistant](https://cla-assistant.io/readme/badge/robotology/idyntree)](https://cla-assistant.io/robotology/idyntree) 
===========

iDynTree is a library of robots dynamics algorithms for control, estimation and simulation.

iDynTree is specifically designed for free-floating robots, but it is possible to use it also  with fixed-base robots.

iDynTree is written in C++ language, but thanks to [SWIG](http://www.swig.org/) it is possible to use the iDynTree algorithms in several other languages. Support and documentation is provided in particular for C++, Matlab, Python and Lua. If you are interested in using iDynTree with another programming language, feel free to [create a new issue](https://github.com/robotology/idyntree/issues/new) requesting support for it.

##  Contents
* **[Installation](#installation)**
* **[Tutorials](#tutorials)**
* **[Documentation](#documentation)**
* **[Announcements](#announcements)**
* **[Developer Documentation](#developer-documentation)**
* **[Reference Paper](#reference-paper)**
* **[Acknowledgments](#acknowledgments)**

## Installation
iDynTree is developed in the context of the [CoDyCo](www.codyco.eu) European Project. For this reason it is usually installed throught the [codyco-superbuild](https://github.com/robotology/codyco-superbuild), an easy way to download, compile and install the CoDyCo software on multiple operating systems, using the [CMake](www.cmake.org) build system and its extension [YCM](http://robotology.github.io/ycm). For more informations on the superbuild concept, please check [YCM documentation](http://robotology.github.io/ycm/gh-pages/master/index.html#superbuild).

If you are not interested in installing all the CoDyCo software it is still possible to install iDynTree without installing the rest of the CoDyCo software. For this reason a superbuild to install iDynTree and all its dependencies is available at:
https://github.com/robotology/idyntree-superbuild . Please check the README of that repo for information on how to install iDynTree and all its dependencies. Note that the `codyco-superbuild` is much more used then the `iDynTree-superbuild`, so it may be easier for you (especially if people working on your lab already use the `codyco-superbuild`) to just use the `codyco-superbuild` for consistency. 

### Bindings
To compile bindings to iDynTree in several scriping languages, you should enable them using the `IDYNTREE_USES_PYTHON`, `IDYNTREE_USES_LUA`, `IDYNTREE_USES_MATLAB`, `IDYNTREE_USES_OCTAVE` CMake options.

Several examples for using the bindigs are available in https://github.com/robotology-playground/idyntree/blob/master/doc/geometric_classes.md .

Then, properly accessing bindings to iDynTree can require some additional steps.
#### Python
You should add to the `PYTHONPATH` enviromental variable the install path of the `iDynTree.py` file.
For a typical installation of the `codyco-superbuild`, this will require adding to the `.bashrc` a line similar to this:
~~~
export PYTHONPATH=$PYTHONPATH:~/src/codyco-superbuild/build/install/lib/python2.7/dist-packages/
~~~

#### Matlab
You should add to Matlab path the `{CMAKE_INSTALL_PREFIX}/mex` directory. By default this directory should be `/usr/local/mex`, or if you used a [YCM](https://github.com/robotology/ycm/) superbuild something like `SUPERBUILD_BUILD_DIRECTORY/install/mex`.
You can modify the installation location for Matlab bindings files using the `IDYNTREE_INSTALL_MATLAB_LIBDIR` and `IDYNTREE_INSTALL_MATLAB_MFILESDIR` CMake options.

#### Octave
You should add to Octave path the `{CMAKE_INSTALL_PREFIX}/octave` directory. By default this directory should be `/usr/local/octave`, or if you used a [YCM](https://github.com/robotology/ycm/) superbuild something like `SUPERBUILD_BUILD_DIRECTORY/install/octave`.
You can modify the installation location for Octave bindings files using the `IDYNTREE_INSTALL_OCTAVE_LIBDIR` and `IDYNTREE_INSTALL_OCTAVE_MFILESDIR` CMake options.


##### Matlab/Octave bindings modifications
All the other bindings (Python,Lua, ...) are generated by SWIG and compiled on the fly by the user,
by enabling the `IDYNTREE_USES_LANGUAGE` option. The Matlab and Octave bindings are an exception because they
rely on an experimental version of Swig, developed for providing Matlab bindings for the [casadi](https://github.com/casadi/casadi/wiki) project. For this reason, usually the Matlab bindigs
are not generated by the users, but by iDynTree developers that have a special experimental Swig
version installed. The bindings code is then committed to the repository, and the `IDYNTREE_USES_MATLAB`
option simply enables *compilation* of the bindings. If you want to regenerate the Matlab bindings,
for example because you modified some iDynTree classes, you can install the experimental
version of Swig with Matlab support at https://github.com/casadi/casadi/wiki/matlab (just follow
the first 4 points in the guide) and then enable Matlab bindings generation with the `IDYNTREE_GENERATE_MATLAB` options.
Currently we are commiting to the repo the matlab bindings generated with this commit of `matlab-swig`: https://github.com/jaeandersson/swig/commit/260ed47c4414e61c66ae84a639707b1fef916ba8 .
For more info on how to modify the matlab bindings, see https://github.com/robotology/idyntree/blob/master/doc/dev/faqs.md#how-to-add-wrap-a-new-class-or-function-with-swig .

#### Dependencies
##### Build dependencies
- [CMake](http://www.cmake.org)

##### Core
- [Eigen](http://eigen.tuxfamily.org)

##### Optional
- [YARP](https://github.com/robotology/yarp)
- [ICUB](https://github.com/robotology/icub-main)
- [Kinematics and Dynamics Library](https://github.com/orocos/orocos_kinematics_dynamics)
- [urdfdom](https://github.com/ros/urdfdom)


## Tutorials
| Topic  | C++ | Matlab | Python |
|:------:|:---:|:------:|:------:|
| Use of the [DynamicsComputation class](http://wiki.icub.org/codyco/dox/html/idyntree/html/classiDynTree_1_1HighLevel_1_1DynamicsComputations.html) for computing Jacobians and Dynamics Regressor | [doc/dcTutorialCpp.md](doc/dcTutorialCpp.md) | NA | NA |
| Use of the [ExtWrenchesAndJointTorquesEstimator class](http://wiki.icub.org/codyco/dox/html/idyntree/html/classiDynTree_1_1ExtWrenchesAndJointTorquesEstimator.html) for computing offset for FT sensors | NA | [examples/matlab/SixAxisFTOffsetEstimation/SixAxisFTOffsetEstimation.m](examples/matlab/SixAxisFTOffsetEstimation/SixAxisFTOffsetEstimation.m) | NA |
| How to get the axis of a revolute joint expressed in a arbitary frame using the [KinDynComputations class](http://wiki.icub.org/codyco/dox/html/idyntree/html/classiDynTree_1_1KinDynComputations.html) | NA | [ examples/matlab/GetJointAxesInWorldFrame.m](examples/matlab/GetJointAxesInWorldFrame.m) | NA | 

Are you interested in a tutorial on a specific feature or algorithm? Just [request it on an enhancement issue](https://github.com/robotology/idyntree/issues/new).

## Documentation
The documentation for the complete API of iDynTree is automatically extracted from the C++ code using [Doxygen](www.doxygen.org),
and is available at the URL : [http://wiki.icub.org/codyco/dox/html/idyntree/html/](http://wiki.icub.org/codyco/dox/html/idyntree/html/).

## Announcements 
Announcements on new releases, API changes or other news are done on [`robotology/QA` GitHub repository](https://github.com/robotology/QA). You can watch that repository to get all the iDynTree-related announcements, that will always tagged with the `announcement` tag.

## Developer Documentation
If you want to contribute to iDynTree development, please check the [Developer's FAQ](https://github.com/robotology/idyntree/blob/master/doc/dev/faqs.md).

## Reference paper
A paper describing the algorithms implemented in iDynTree and their use in a real world scenario can be downloaded [here](http://journal.frontiersin.org/article/10.3389/frobt.2015.00006/abstract) . If you're going to use this library for your work, please quote it within any resulting publication:
~~~
F. Nori, S. Traversaro, J. Eljaik, F. Romano, A. Del Prete, D. Pucci "iCub whole-body control through force regulation on rigid non-coplanar contacts", Frontiers in Robotics and AI, 2015.
~~~

The bibtex code for including this citation is provided:
~~~
@ARTICLE{10.3389/frobt.2015.00006,
  AUTHOR={Nori, Francesco  and  Traversaro, Silvio  and  Eljaik, Jorhabib  and  Romano, Francesco  and  Del Prete, Andrea  and  Pucci, Daniele},
  TITLE={iCub Whole-body Control through Force Regulation on Rigid Noncoplanar Contacts},
  JOURNAL={Frontiers in Robotics and AI},
  VOLUME={2},
  YEAR={2015},
  NUMBER={6},
  URL={http://www.frontiersin.org/humanoid_robotics/10.3389/frobt.2015.00006/abstract},
  DOI={10.3389/frobt.2015.00006},
  ISSN={2296-9144}}
~~~

## Acknowledgments
The development of iDynTree is supported by the FP7 EU projects [CoDyCo (No. 600716 ICT 2011.2.1 Cognitive
Systems and Robotics)](http://www.codyco.eu/)  and [Koroibot (No. 611909 ICT- 2013.2.1 Cognitive Systems and Robotics)](http://koroibot.eu/).
The development is also supported by the [Istituto Italiano di Tecnologia](http://www.iit.it).

## License 
iDynTree is licensed under either the GNU Lesser General Public License v3.0 : 

https://www.gnu.org/licenses/lgpl-3.0.html

or the GNU Lesser General Public License v2.1 :

https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html

at your option.

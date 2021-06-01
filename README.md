# lynx

![version](https://img.shields.io/badge/version-0.1.0-blue)
![example workflow](https://github.com/uwgraphics/lynx/actions/workflows/ci.yml/badge.svg)

Welcome to the Lynx Robotics Library!  The library will feature useful robotics applications, such as forward kinematics, inverse kinematics, path planning, trajectory optimization, and pathwise inverse kinematics.  It was designed to be general purpose and easy to set up, going straight from robot URDF to modeling, planning, and optimization within minutes.  


<h1> v0.1.0 Release Notes </h1>

Lynx v0.1.0 provides some base features for robot modeling and planning.  The library is under heavy development, and many more features will be released in v0.2 by July 1 (outlined below).  If you would like to try out the library in its current state, refer to the Examples in lynx_rust/examples.


<h1> Plans for v0.2.0 Release </h1>

The following features will be available in our Lynx v0.2.0 release by July 1, 2021:

* The library will feature a built-in graphics engine with physically-based lighting and rendering, making it easy to interface with many robots for planning and motion optimization.
* Many more robotics algorithms will be implemented in the library, including RelaxedIK (published at RSS 2018), CollisionIK (published at ICRA 2021), and Strobe (published at ICRA 2021).   
* The code will be better documented and the library as a whole will have more thorough instructions for setup and use.
* A seperate repository will be released that will contain dozens of pre-processed robots and environments to use out of the box for any of our implemented algorithms.

<h1> Plans for v0.3.0 and Beyond </h1>

* We plan to release ROS wrappers that will easily bridge the gap between Lynx and other robotics software stacks.  
* We will continue to add more robot motion planning and optimization algorithms, e.g., Stampede (published at ICRA 2021) with corresponding apps within our graphics engine.



If you have any questions, feel free to post an issue or email me at rakita@cs.wisc.edu




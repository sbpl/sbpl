

Compilation:

a) Under windows, SBPL can be compiled using a project inside sbpl\win32_build\test_vs2007. 
   This supports Visual Studio 7.0. Other projects in wind32_build directory may be
   outdated and may therefore require you to add new .cpp files to the project.


b) Under unix/linux, you can either 
 b.1) run "cmake CMakeLists.txt" in sbpl\cmake_build and then "make" in the same directory
      (note: don't try to run cmake in sbpl. CMakeLists.txt file in there is for ROS make)
 b.2) or create your own makefile.

 
c) For those using SBPL as part of ROS:

You can download the whole ROS package (see http://www.ros.org/wiki/ROS/Installation) and just do "rosmake sbpl" within sbpl directory.


Usage:

Examples for how to use SBPL is in src/test/main.cpp. 
Please follow the examples carefully - it will save you a lot of debugging time since currently there is no documentation available for the library. The library contains a number of planning problem examples, stored as ascii files. These files (with extension .cfg) should be passed in as arguments into the main function in main.cpp. The files can be found in env_examples directory.

Note: in main function in main.cpp, you can comment and uncomment the planners you want to run.
The xytheta lattice planners require two arguments: cfg file and mprim file. The latter specifies the
motion primitives according to which the robot can move in x,y,theta. Motion primitives files can be found in sbpl/matlab/mprim directory


Finally, few visualization scripts can be found in sbpl/matlab/visualization. In particular, plot_3Dpath.m function can be used to visualize the path found by xytheta lattice planner. This functions takes in .cfg file that specified environment and sol.txt file that was generated within main.cpp by xythetalattice planners.


Links:
For more information and documentation on SBPL visit:
http://www.ros.org/wiki/sbpl

For more information and documentation on using the x,y,theta environment under ROS visit:
http://www.ros.org/wiki/sbpl_lattice_planner



Compilation:

a) Under windows, SBPL can be compiled using a project inside sbpl\win32_build\test_vs2007. 


b) Under unix/linux, you can either use cmake in sbpl\cmake_build or create your own makefile.

 
c) For those using SBPL as part of ROS:

You can download the whole ROS package (see http://www.ros.org/wiki/ROS/Installation) and just do "rosmake sbpl" within sbpl directory.


Usage:

Examples for how to use SBPL is in src/test/main.cpp. Please follow the examples carefully - it will save you a lot of debugging time 
since currently there is no documentation available for the library. The library contains a number of planning problem examples, 
stored as ascii files. These files (with extension .cfg) should be passed in as arguments into the main function in main.cpp. 
The files can be found in env_examples directory.

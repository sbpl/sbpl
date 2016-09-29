I. Building, Installing, and Using SBPL

    SBPL is available as a standalone software library. SBPL itself has no
    dependencies other than the C/C++ standard library.

    These build and install instructions are primarily for Linux. For other
    operating systems, CMake can generate the platform-specific build and project
    files necessary for building SBPL.

    Versions of ROS older than Fuerte may contain packages that depend on a ROS
    package version of SBPL. The recommended method to install SBPL is to install
    it as a standard system library. However, if you wish to use the old ROS
    package version of SBPL, you may follow these instructions.

    1. Building and Installing SBPL from source

        1.1 Build SBPL

            SBPL uses git as its version control system. From the directory where
            you want the SBPL source to reside, clone the latest source from
            https://github.com/sbpl/sbpl:

            git clone https://github.com/sbpl/sbpl.git

            In the source directory, build the SBPL library using standard
            CMake build conventions:

            mkdir build
            cd build
            cmake ..
            make

        1.2 Install SBPL

            Install the built library and headers onto your local system
            (usually into /usr/local):

            sudo make install

    2. Installing SBPL from pre-built binary package

        A pre-built Debian package exists on Linux for ROS distributions
        Fuerte and newer. To install the Debian, run:

        sudo apt-get install ros-distro-sbpl

        where distro is the name of your ROS distribution. This will install
        the SBPL library and associated development headers alongside other
        ROS components (in /opt/ros/distro on Ubuntu distributions). A
        pkg-config file is also included to allow you to locate the SBPL
        library components in your build system.

    3. Build your (ROS) package with SBPL as a dependency (CMake)

        In the CMakeLists.txt for your (ROS) package, the following lines are
        needed to find the installed SBPL files:

        find_package(PkgConfig REQUIRED)
        pkg_check_modules(SBPL REQUIRED sbpl)
        include_directories(${SBPL_INCLUDE_DIRS})
        link_directories(${SBPL_LIBRARY_DIRS})

        Then, after you've declared your binaries, you need to link them
        against SBPL with the following line:

        target_link_libraries(your-binary-here ${SBPL_LIBRARIES})

    4. Installing and Using SBPL as a ROS package

        The ROS package version of SBPL was deprecated with the release of ROS
        Fuerte. However, packages in ROS Electric may still require the ROS
        package version of SBPL.

        4.1 Install SBPL

            4.1.1 Source install

                SBPL uses git as its version control system. From the
                directory where you want the SBPL source to reside, clone the
                latest source from https://github.com/sbpl/sbpl:

                git clone https://github.com/sbpl/sbpl.git

                In the source directory, checkout the electric branch of the
                repository to revert to the old ROS package version:

                git checkout -b electric

                Ensure that SBPL is on your ROS_PACKAGE_PATH and type:

                rosmake sbpl

            4.1.2 Binary install

                SBPL is also available as a pre-built Debian in ROS Electric.
                To instal the Debian, run:

                sudo apt-get install ros-electric-arm-navigation

        4.2 Build your ROS package with SBPL as a depency (rosbuild)

            In the manifest.xml for your package, you need to add the
            following line to declare the SBPL package as a dependency:

            <depend package="sbpl"/>

II. Usage

    Examples for how to use SBPL are in src/test/main.cpp.  Please follow the
    examples carefully. The library contains a number of planning problem
    examples, stored as ascii files.  These files (with extension .cfg) should
    be passed in as arguments into the main function in main.cpp. The files
    can be found in env_examples directory.

    Command-line usage for the test_sbpl program can be viewed by passing '-h'
    as argument to the executable.

    Examples:

        The following can be run from the directory containing test_sbpl,
        which we assume is a build directory in the root of this project.

        $ ./test_sbpl ../env_examples/nav3d/env1.cfg
        Environment: xytheta; Planner: arastar; Search direction: backward
        Initializing ARAPlanner...
        start planning...
        done planning
        size of solution=16
        solution size=0
        Solution is found

        $ ./test_sbpl --env=2d ../env_examples/nav2d/env1.cfg #2d is needed here in order to use 2d config
        Environment: 2d; Planner: arastar; Search direction: backward
        Initializing ARAPlanner...
        start planning...
        done planning
        size of solution=22
        Solution is found

        $ ./test_sbpl --env=robarm --search-dir=forward --planner=rstar ../env_examples/robarm/env1_6d.cfg
        Environment: robarm; Planner: rstar; Search direction: forward
        Initializing RSTARPlanner...
        start planning...
        done planning
        size of solution=44
        Solution is found

   Motion primitives files can be found in sbpl/matlab/mprim directory.

    Finally, few visualization scripts can be found in
    sbpl/matlab/visualization. In particular, plot_3Dpath.m function can be
    used to visualize the path found by xytheta lattice planner. This
    functions takes in .cfg file that specified environment and sol.txt file
    that was generated within main.cpp by xythetalattice planners.

    Note: If you compile the library with the ROS symbol defined, all text
    output will be redirected to ROS logging constructions. Without the ROS
    symbol defined, SBPL will print messages to stdout and test_sbpl will
    generate a solution file, sol.txt, as well as a debugging information
    file, debug.txt

III. Links

    These instructions and more tutorials can be found at www.sbpl.net

    For more information and documentation on SBPL visit:

    http://www.ros.org/wiki/sbpl

    For more information and documentation on using the x,y,theta environment
    available in ROS visit:

    http://www.ros.org/wiki/sbpl_lattice_planner

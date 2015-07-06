^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sbpl-cpr
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.1 (2015-07-06)
------------------
* Added version to package.
* Added catkin related file and changed destination name/dirs from sbpl to sbpl-cpr
* Optional memory saving features related to SBPL2DGridSearch. Can save many GB of memory for large grids.
  Added feature: Search the 2D heuristic at a different resolution than the 3D main grid for EnvironmentNAVXYTHETALATTICE. Default is a 1:1 correspondence as before but different resoltions available with Set2DBlockSize.
  Added feature: Allow 2D priority queue to have dynamically allocated memory for the CSlidingBucket. Default action is fixed/worst case as before but setting an explicit initial bucket size with Set2DBucketSize will allow the buckets to grow.
* add compute_suboptimality method to ara*
* Merge pull request `#28 <https://github.com/clearpathrobotics/sbpl/issues/28>`_ from shoppel/master
  Adds a method to request the used actions in EnvironmentNAVXYTHETALAT.
* Adds a method to request the used actions.
* Merge branch 'master' of https://github.com/sbpl/sbpl
* Adds constructors in struct ENV_NAV2D_CONFIG and ENVNAV2D and sets the contained pointers to NULL.
  Prevents the destructor from using wild pointers for deletion.
* Contributors: Andrew Dornbush, Jason Mercer, SBPL, Stefan Haase

1.2.0 (2014-08-08)
------------------
* bump version to 1.2.0
* Merge pull request `#24 <https://github.com/clearpathrobotics/sbpl/issues/24>`_ from sbpl/dev
  Merge of dev into master
* Merge pull request `#23 <https://github.com/clearpathrobotics/sbpl/issues/23>`_ from sbpl/master
  Merge of master into dev
* Merge pull request `#22 <https://github.com/clearpathrobotics/sbpl/issues/22>`_ from robotman0/dev
  Bug fix in LazyARA. It also now fully supports the planner API.
* Bug fix in LazyARA. It also now fully supports the planner API.
* Merge pull request `#21 <https://github.com/clearpathrobotics/sbpl/issues/21>`_ from robotman0/dev
  Updated the environment API to support lazy and egraph planners. Added LazyARA*.
* Updated the environment API to support lazy and egraph planners. Added a LazyARA* planner.
* Merge branch 'dev'
* Merge branch 'dev' of https://github.com/sbpl/sbpl into dev
* Merge branch 'master' into dev
* Merge pull request `#20 <https://github.com/clearpathrobotics/sbpl/issues/20>`_ from bmacallister85/dev_ad_planner_h_bug
  adplanner bug fixes
* Merge pull request `#19 <https://github.com/clearpathrobotics/sbpl/issues/19>`_ from ahornung/master
  Adding release version number to CMakeList, pkg-config, and cmake-config
* adplanner:   Reinsert check for search efforts in costs_changed method and set bReevalutefvals flag.
* Proposed AD* changes
  1.)  Update heuristic values for all states when SearchGoal or edgeCosts have changed.
  2.)  Couple new search iteration with merging of incons list
  3.)  Do not make new search iteration just because the start or goal was set (may not have changed)
* Adding release version number to CMakeList, pkg-config, and cmake-config
* correct ros define in config header
* disable debug output unless DEBUG flag is set
* Merge pull request `#18 <https://github.com/clearpathrobotics/sbpl/issues/18>`_ from ahornung/master
  Adding a CMake config
* Separating generated pkgconfig file from source dir, is now put into build dir before installing
* Adding a CMake config
  This enables finding and linking against the library with the preferred
  CMake mechanism (as opposed to piggybacking on pkgconfig).
  The absolute path to the library enables multiple
  installations side-by-side, you can point CMake to the one you want to
  use with the env.var sbpl_DIR.
* Merge pull request `#16 <https://github.com/clearpathrobotics/sbpl/issues/16>`_ from bmacallister85/origin/dev-ara-redunant-flag-remove
  Origin/dev ara redunant flag remove
* test `#1 <https://github.com/clearpathrobotics/sbpl/issues/1>`_ commit
* Merge pull request `#15 <https://github.com/clearpathrobotics/sbpl/issues/15>`_ from bmacallister85/dev
  Dev
* araplanner: removed redundant bRebuildOpenList flag
* Merge remote-tracking branch 'refs/remotes/sbpl/master' into dev
* Merge remote-tracking branch 'refs/remotes/sbpl/dev' into dev
* Merge branch 'master' of https://github.com/sbpl/sbpl
* Update documentation for get_n_expands_init_solution
* return correct inital solution expansions when planner fails
* Merge pull request `#14 <https://github.com/clearpathrobotics/sbpl/issues/14>`_ from bmacallister85/origin/ara-timeout-fix-improvement
  araplanner:
* Merge pull request `#6 <https://github.com/clearpathrobotics/sbpl/issues/6>`_ from bmacallister85/origin/dev2
  environment_navxythetamlevlat.cpp: Removed out of bounds indexing call i...
* araplanner:
  Moved reinitialization of goal state to inside ReInitializeSearchStateSpace
* remove outdated vs project files; use cmake to generate these
* update and prettify the README to reflect current build steps
* Merge pull request `#12 <https://github.com/clearpathrobotics/sbpl/issues/12>`_ from bmacallister85/origin/ara-bug-fix
  Bug fix for araplanner
* Bug fix for araplanner
* Merge pull request `#11 <https://github.com/clearpathrobotics/sbpl/issues/11>`_ from bmacallister85/master-motion-prim-corrections
  Master motion prim corrections
* Removed misplaced character.
* Motion primitive scripts can now produce longer rotation in place motions.
* Merge pull request `#9 <https://github.com/clearpathrobotics/sbpl/issues/9>`_ from bmacallister85/origin/master2
  sbpl message macros changed to allow the use of a special function to ha...
* Merge pull request `#10 <https://github.com/clearpathrobotics/sbpl/issues/10>`_ from bmacallister85/origin/master-motion-prim-corrections
  For all motion primitive scripts and files, corrected rotate in place m...
* Merge pull request `#8 <https://github.com/clearpathrobotics/sbpl/issues/8>`_ from bmacallister85/origin/dev4
  utils.cpp: Set default values for array in PathExists method.  Added del...
* Merge pull request `#7 <https://github.com/clearpathrobotics/sbpl/issues/7>`_ from bmacallister85/origin/dev3
  environment_roboarm.cpp: Removed unused variable and conditional in Prin...
* For all motion primitive scripts  and files, corrected rotate in place motions between last and first angles.
* sbpl message macros changed to allow the use of a special function to handle output at different levels similar to ROS.
* utils.cpp: Set default values for array in PathExists method.  Added deletes in EvaluatePolicy method.
* environment_roboarm.cpp: Removed unused variable and conditional in PrintSate method.
* environment_navxythetamlevlat.cpp: Removed out of bounds indexing call in GetActionCostacrossAddLevels method.
* Merge pull request `#5 <https://github.com/clearpathrobotics/sbpl/issues/5>`_ from bmacallister85/master
  2dGridSearch delete/free mismatch
* 2dgridsearch.cpp: Replaced delete with free  in SBPL2DGridSearch::search_withslidingbuckets method.
* fix bug caused by heuristic computation taking up all allocated time
* add flags to ARA* to indicate when the open and incons lists should be merged
* apply goal heuristics bug fix for case where costs change
* clean up goal heuristics bug fix
* ensures heuristics are updated before setting the search goal state
* clear output vector before generating footprint cells for additional levels in multi-lev
* change default build to release
* Contributors: Andrew Dornbush, Armin Hornung, Brian MacAllister, Margarita Safonova, Mike Phillips, SBPL, aurone

1.1.4 (2013-01-07)
------------------
* add c lib include to disambiguate abs
* Contributors: Andrew Dornbush

1.1.3 (2012-11-08)
------------------
* fix compilation with DEBUG flag on
  fix compilation with DEBUG and TIME_DEBUG flags set to 1
  fix cast to ARAState in adplanner.cpp
  remove more unnecessary headers
* minimize dependencies between source files
* standardize formatting of header files
* standardize formatting for test source files
* standardize formatting of utils sources files
* standardize formatting in planner source files
* standardize formatting in env source files
* Fixed segfault due to NULL ptr access in printf
* Added member variables and setters for #define constants
* Contributors: Andrew Dornbush, Armin Hornung

1.1.2 (2012-08-17 14:09)
------------------------
* Test space deletion for release cycle
* Contributors: egiljones

1.1.1 (2012-08-17 13:18)
------------------------
* Removing ROS references in CMakeLists.txt, they'll be patched in downstream
* Merge branch 'reorg'
* fixed segfault in RSTARPlanner::SetBestPredecessor()
* Contributors: Andrew Dornbush, egiljones

1.1.0 (2012-06-08)
------------------
* committing
* Updating to do things the right way with Ioan's help, getting rid of catkin dependency and manifest, and moving stack.yaml to external repo
* Adding stack.yaml for catkin
* Making a bunch of changes that make the library more useful once installed using catkin.
* Re-added more ROS-appropriate Makefile
* Adding Visual Studio 2010 project files
* Many various additions and improvements
  Optimized footprint calculation
  Added ReplanParams and PlannerStats classes
  Added ways to call replan
  Added main function and command-line interface
  Made functions in base planner class polymorphic
  Added test script to run through various SBPL examples
  Generated 10cm PR2 motion primitives
  Moved old SVN history into new Git repo
  Changed all SBPL_PRINTFs to normal printfs in main
* fixed some compile warnings
* added ANA* search
* fixed few bugs found by Dustin Geletko and myself in R* planner
* few small changes. One bug in main.cpp in planandnavigate2d function
* Merging r49786 through r49804 into trunk
* unstable is now trunk
* fixed a small bug in how the path is reconstructed in ARA* and AD*. Also fixed a compilation flag that caused a debug file to be opened but not closed.
* added SBPL_DEBUG_NAMED
* -added some printf defines to resemble the ROS logging structure (so now there is SBPL_INFO(=ROS_INFO),SBPL_DEBUG,SBPL_WARN....)
  -left in SBPL_PRINTF
* modified matlab scripts for making motion primitives a little easier
* Replaced printf,fprintf,fopen,fclose,fflush function calls with SBPL_XXX macros for ROS and non-ROS builds. Replaced all exit calls with exceptions so that higher level code can catch and handle them. Fixed all warnings. Matched each call to fopen with a corresponding fclose. Applied statistic getting functions from ARA* to be in the interface for all planners (though it is only implemented for ARA* and AD*).
* fixed some bugs in the support for multi-level 3D planning
* fixed cmake files for independent compilation of sbpl under linux and also updated README.txt files with some additional comments
* added README.txt with few notes
* support for multi-level 3D (x,y,theta) planning
* Merging over from multidof2
* added more debugging access functions
* Committing major changes from multidof, with the exception of to the ik_constrained_planner
* Merging ompl build-directory fix into trunk
* Now opens debug file in tmp so that it can work on computers with a shared install
* Added Ubuntu platform tags to manifest
* fixed a bug in sliding buckets that are used in 3D lattice planning
* preparing motion_planners 0.2.0
* converted comments to doxygen
* comments
* comments
* comments
* comments
* comments
* comments
* comments
* comments
* comments
* comments
* comments
* comments
* 
* entered comments
* working on comments
* adding comments
* staging motion_planners into tick-tock
* Contributors: Andrew Dornbush, bcohen, egiljones, eitan, gerkey, gjones, kwc, leibs, maximl, miph, sachinc, unknown

cmake_minimum_required(VERSION 2.4.6)
set(CMAKE_BUILD_TYPE Release)
include_directories(${PROJECT_SOURCE_DIR}/code)

add_library(sbpl
  ../src/discrete_space_information/nav2d/environment_nav2D.cpp
  ../src/discrete_space_information/navxythetalat/environment_navxythetalat.cpp
  ../src/discrete_space_information/navxythetalat/environment_navxythetamlevlat.cpp
  ../src/discrete_space_information/nav2d_uu/environment_nav2Duu.cpp
  ../src/discrete_space_information/template/environment_XXX.cpp
  ../src/discrete_space_information/robarm/environment_robarm.cpp
  ../src/planners/ARAStar/araplanner.cpp
  ../src/planners/ADStar/adplanner.cpp
  ../src/planners/VI/viplanner.cpp
  ../src/planners/PPCP/ppcpplanner.cpp
  ../src/planners/RStar/rstarplanner.cpp
  ../src/utils/heap.cpp
  ../src/utils/mdp.cpp
  ../src/utils/utils.cpp
  ../src/utils/2Dgridsearch.cpp
  )

# Build executables in the bin directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# rosbuild_add_executable(test_sbpl src/test/main.cpp)
add_executable(test_sbpl ../src/test/main.cpp)
target_link_libraries(test_sbpl sbpl)

# rosbuild_add_executable(test_adjacency_list src/test/test_adjacency_list.cpp)
add_executable(test_adjacency_list ../src/test/test_adjacency_list.cpp)
target_link_libraries(test_adjacency_list sbpl)

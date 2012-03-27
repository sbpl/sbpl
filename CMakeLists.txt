cmake_minimum_required(VERSION 2.4.6)
PROJECT( sbpl )
find_package(catkin)

catkin_stack()
catkin_project(sbpl
  LIBRARIES sbpl
  INCLUDE_DIRS sbpl
  )	

if(WITH_ROS)
  # Add a ROS C++ macro to tell SBPL to use ROS print statements
  add_definitions(-DROS)  
endif(WITH_ROS)

include_directories(src/include)

add_library(sbpl SHARED
  src/discrete_space_information/environment_nav2D.cpp
  src/discrete_space_information/environment_navxythetalat.cpp
  src/discrete_space_information/environment_navxythetamlevlat.cpp
  src/discrete_space_information/environment_nav2Duu.cpp
  src/discrete_space_information/environment_XXX.cpp
  src/discrete_space_information/environment_robarm.cpp
  src/planners/araplanner.cpp
  src/planners/adplanner.cpp
  src/planners/viplanner.cpp
  src/planners/ppcpplanner.cpp
  src/planners/ANAplanner.cpp
  src/planners/rstarplanner.cpp
  src/utils/heap.cpp
  src/utils/mdp.cpp
  src/utils/utils.cpp
  src/utils/2Dgridsearch.cpp	
  )

install(DIRECTORY src/include/sbpl DESTINATION include
	FILES_MATCHING PATTERN "*.h")
install(TARGETS sbpl
	LIBRARY DESTINATION lib)
install(FILES manifest.xml
	DESTINATION share/sbpl)

add_executable(test_sbpl src/test/main.cpp)
target_link_libraries(test_sbpl sbpl)

add_executable(test_adjacency_list src/test/test_adjacency_list.cpp)
target_link_libraries(test_adjacency_list sbpl)

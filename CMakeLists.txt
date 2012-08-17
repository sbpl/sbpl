cmake_minimum_required(VERSION 2.8)
PROJECT( sbpl )

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

set(PKG_NAME "sbpl")
set(PKG_DESC "The Search Based Planning Library")
set(PKG_SBPL_LIBS "-lsbpl")
set(pkg_conf_file "${CMAKE_CURRENT_SOURCE_DIR}/sbpl.pc")
configure_file("${pkg_conf_file}.in" "${pkg_conf_file}" @ONLY)
install(FILES ${pkg_conf_file} DESTINATION lib/pkgconfig/ COMPONENT pkgconfig)

install(DIRECTORY src/include/sbpl DESTINATION include
	FILES_MATCHING PATTERN "*.h")
install(TARGETS sbpl
	LIBRARY DESTINATION lib)

add_executable(test_sbpl src/test/main.cpp)
target_link_libraries(test_sbpl sbpl)

add_executable(test_adjacency_list src/test/test_adjacency_list.cpp)
target_link_libraries(test_adjacency_list sbpl)

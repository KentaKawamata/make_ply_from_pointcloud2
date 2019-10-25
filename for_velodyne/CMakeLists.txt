cmake_minimum_required(VERSION 2.8.3)
project(superimpose_velo_points)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(USE_PCL 1 "Find and link Point Cloud Library")
set(USE_EIGEN 1 "Find and link Eigen Library")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  std_msgs
  tf2_ros
)

if(USE_PCL)
    find_package(PCL REQUIRED)

    if(PCL_FOUND)
        message("---- Found PCL ${PCL_INCLUDE_DIRS}")

    else(PCL_FOUND)
        message(FATAL_ERROR "---- Not Found PCL")

    endif(PCL_FOUND)
endif(USE_PCL)

if(USE_EIGEN)
    find_package(Eigen3 REQUIRED)

    if(Eigen3_FOUND)
        message("---- Found Eigen ${Eigen3_INCLUDE_DIRS}")

    else(Eigen3_FOUND)
        message(FATAL_ERROR "---- Not Found Eigen")

    endif(Eigen3_FOUND)
endif(USE_EIGEN)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs pcl_conversions pcl_ros roscpp std_msgs tf2_ros
  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ros_to_pcl.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(superimpose_velo_points
  src/make_pc.cpp
  include/make_pc.hpp
  src/editCloud.cpp
  include/editCloud.hpp
  src/getRotationVector.cpp
  include/getRotationVector.hpp
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(superimpose_velo_points
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(superimpose_velo_points
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Eigen3_LIBRARIES}
)

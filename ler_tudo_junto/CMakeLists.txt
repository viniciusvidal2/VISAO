cmake_minimum_required(VERSION 2.8.3)
project(ler_tudo_junto)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp sensor_msgs pcl_ros mavros mavros_msgs cv_bridge mavlink std_msgs image_transport
)

find_package(OpenCV REQUIRED)

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
#  INCLUDE_DIRS include
#  LIBRARIES ler_tudo_junto
#  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros mavros mavros_msgs cv_bridge zed_wrapper
#  DEPENDS system_lib zed_wrapper
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../libraries
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(ler_tudo_junto_node src/ler_tudo.cpp)
target_link_libraries(ler_tudo_junto_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)
#add_dependencies(ler_tudo_junto_node ${ler_tudo_junto_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

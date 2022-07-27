cmake_minimum_required(VERSION 3.0.2)
project(planner_to_px4)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  catkin_simple
  # roscpp
  # mavlink
  # mavros
  # mavros_extras
  # mavros_msgs
)

catkin_simple()


# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES slower_msg slower_trad
#   CATKIN_DEPENDS mavros mavros_extras mavros_msgs roscpp geometry_msgs trajectory_msgs
# )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
#   ${Boost_INCLUDE_DIR}
#   ${Eigen_INCLUDE_DIRS}
# )


## Declare a C++ library


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
cs_add_library(slower_msg src/slower_msg.cpp)
cs_add_executable(slower_msg_node src/slower_msg_node.cpp)
target_link_libraries(slower_msg_node slower_msg)

cs_add_library(slower_trad src/slower_trad.cpp)
cs_add_executable(slower_trad_node src/slower_trad_node.cpp)
target_link_libraries(slower_trad_node slower_trad)

cs_install()
cs_export()

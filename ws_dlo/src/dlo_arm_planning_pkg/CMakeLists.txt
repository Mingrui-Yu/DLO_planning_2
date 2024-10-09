cmake_minimum_required(VERSION 3.0.2)
project(dlo_arm_planning_pkg)

# colors, because why not :)
string(ASCII 27 Esc)
set(RED "${Esc}[31m")
set(GREEN "${Esc}[32m")
set(ENDL "${Esc}[m")

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()


set(CMAKE_CXX_FLAGS "-std=c++14 -O3") # seems that c++17 doesn't work.

# CMAKE_MODULE_PATH
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

if(NOT DEVICE_ID)
    set(DEVICE_ID 0)
endif()


MESSAGE("${GREEN}----------------Options:-------------------${ENDL}")
MESSAGE("${GREEN}Build type: ${CMAKE_BUILD_TYPE}${ENDL}")
MESSAGE("${GREEN}DEVICE_ID: ${DEVICE_ID}${ENDL}")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  octomap_msgs
  roscpp
  rospy
  rviz_visual_tools
  std_msgs
  visualization_msgs
)

# System dependencies are found with CMake's conventions
find_package(Boost REQUIRED system filesystem date_time thread)

# Eigen
include_directories("/usr/include/eigen3") # custom installed eigen 3.4.0

# ifopt
find_package(ifopt)

# ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# cnpy
include_directories("/usr/local/cnpy.h")

# # GeometricTools
# set(GTE_INC_DIR "/home/mingrui/Mingrui/software/GeometricTools/GTE")
# include_directories(${GTE_INC_DIR})

# FCL
find_package(FCL REQUIRED)

# yaml-cpp
find_package(yaml-cpp REQUIRED)

# # for using casadi
# find_package(casadi REQUIRED)
# set(CASADI_INCLUDE_DIR /usr/local/include/casadi)
# include_directories(${CASADI_INCLUDE_DIR})
# # find casadi library
# find_library(CASADI_LIBRARY
#     NAMES casadi
#     HINTS ${CASADI_INCLUDE_DIR}/../lib $ENV{CASADI_PREFIX}/lib)
# if(CASADI_LIBRARY)
#     set(CASADI_LIBRARIES ${CASADI_LIBRARIES} ${CASADI_LIBRARY})
# endif()
# # For multi threading
# find_package(Threads REQUIRED)



## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   octomap_msgs#   std_msgs#   visualization_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
#  LIBRARIES dlo_arm_planning_pkg
#  CATKIN_DEPENDS geometry_msgs message_generation moveit_core moveit_ros_planning moveit_ros_planning_interface moveit_visual_tools octomap_msgs roscpp rospy rviz_visual_tools std_msgs visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(THIRD_PARTY_LIBS
    ${catkin_LIBRARIES}
    ${ifopt_LIBRARIES}
    ${CERES_LIBRARIES}
    ${CNPY_LIBRARIES}
    cnpy z # for cnpy & (zlib)
    ${FCL_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    # ${CASADI_LIBRARIES}
)

# Declare a C++ library
add_library(DLO_ARM_PLANNING_LIBS
  src/dlo_state.cpp
  src/dlo.cpp
  src/arm.cpp
  src/arm_reach_space.cpp
  src/dual_arm.cpp 
  src/scene.cpp 
  src/visualize.cpp 
  src/ur5.cpp
  src/collision_detection_fcl.cpp
  src/planning_interface.cpp
  src/planner/planner_base.cpp 
  src/planner/joint_birrt.cpp
  src/real_time_interface.cpp
  src/controller.cpp
  src/jacobian_model.cpp
  src/mpc/mpc_base.cpp
  src/mpc/multi_step_mpc.cpp
)

target_link_libraries(DLO_ARM_PLANNING_LIBS
  ${catkin_LIBRARIES}
  ${THIRD_PARTY_LIBS}
)



## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

add_executable(example_vis_derm_projection app/example_vis_derm_projection.cpp)
target_link_libraries(example_vis_derm_projection
    ${THIRD_PARTY_LIBS}
    DLO_ARM_PLANNING_LIBS
)

add_executable(example_planner_without_unity app/example_planner_without_unity.cpp)
target_link_libraries(example_planner_without_unity
    ${THIRD_PARTY_LIBS}
    DLO_ARM_PLANNING_LIBS
)

add_executable(eval_planner app/eval_planner.cpp)
target_link_libraries(eval_planner
    ${THIRD_PARTY_LIBS}
    DLO_ARM_PLANNING_LIBS
)

add_executable(eval_controller app/eval_controller.cpp)
target_link_libraries(eval_controller
    ${THIRD_PARTY_LIBS}
    DLO_ARM_PLANNING_LIBS
)
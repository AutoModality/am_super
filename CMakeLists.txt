cmake_minimum_required(VERSION 2.8.3)
project(am_super)

## Compile as C++11, supported in ROS melodic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  am_utils
  diagnostic_msgs
  rosbag
  roscpp
  rospy
  std_msgs
  std_srvs
)

find_package(CUDA)

if(CUDA_FOUND)
	add_definitions(-DCUDA_FLAG)
	message([STATUS] "CUDA FOUND, am_super will be built with GPU monitoring")
	SET(CUDA_NVCC_FLAGS "-arch=sm_62" CACHE STRING "nvcc flags" FORCE)
	SET(CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
	CUDA_ADD_LIBRARY(cuda_utility ${LIB_TYPE} src/cuda/cuda_utility.cu)
endif(CUDA_FOUND)


## System dependencies are found with CMake's conventions
find_package(DJIOSDK REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################
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
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

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
  INCLUDE_DIRS include
  LIBRARIES super_lib
  CATKIN_DEPENDS 
  am_utils
  diagnostic_msgs
  rosbag
  roscpp
  rospy
  std_msgs
  std_srvs
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

## Declare a C++ library
add_library(super_lib
  src/super_lib/am_life_cycle.cpp
)
target_link_libraries(super_lib ${catkin_LIBRARIES})
add_dependencies(super_lib ${catkin_EXPORTED_TARGETS})

if(CUDA_FOUND)
	add_executable(am_super src/am_super/am_super.cpp src/cuda/cuda_utility_class.cpp)
	target_link_libraries(am_super ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES} super_lib cuda_utility)
else()
	add_executable(am_super src/am_super/am_super.cpp)
	target_link_libraries(am_super ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES} super_lib)
endif(CUDA_FOUND)
add_dependencies(am_super ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############
# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#  scripts/my_python_script
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
 install(TARGETS ${PROJECT_NAME}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

# Mark libraries for installation
# See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
 install(TARGETS super_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
 )

#Mark cpp header files for installation
install(DIRECTORY include/super_lib
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_machine_learning.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

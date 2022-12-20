cmake_minimum_required(VERSION 3.5)
project(am_super)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #to view [-Wunused-parameter] uncomment the following line
  #add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(brain_box_msgs REQUIRED)
find_package(am_utils REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED) 

set(dependencies
        brain_box_msgs
        am_utils
        builtin_interfaces
        std_msgs
        rosbag2
        rclcpp
        rclpy
        rosbag2_cpp
        diagnostic_msgs
        diagnostic_updater
      )

include_directories(
		include
  	  )


file(GLOB super_lib_cpp_files
    src/super_lib/*.cpp
)

file(GLOB cuda_cpp_files
    src/cuda/*.cpp
)

file(GLOB am_super_cpp_files
    src/am_super/*.cpp
)

file(GLOB SUPER_MEDIATOR_FILES
    src/am_super/*_mediator.cpp
)

add_library(super_lib
  ${super_lib_cpp_files}
)
target_link_libraries(super_lib)

ament_target_dependencies(super_lib ${dependencies})
ament_export_targets(super_libTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(super_lib)


#add_executable(am_super ${am_super_cpp_files})
#target_link_libraries(am_super super_lib)
#ament_target_dependencies(am_super ${dependencies})


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
install(DIRECTORY include/
        DESTINATION include/
    )
install(
  TARGETS super_lib
  EXPORT super_libTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
#install(TARGETS
#  am_super
#  DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(am_utils)
ament_export_dependencies(${dependencies})
ament_package()

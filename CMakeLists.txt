cmake_minimum_required(VERSION 3.5)
project(am_super)

set(CMAKE_BUILD_TYPE Debug)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(dependencies
  am_utils
  brain_box_msgs
  builtin_interfaces
  rclcpp
  rclpy
  rosbag2
  rosbag2_cpp
  std_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
	include
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

add_executable(am_super ${am_super_cpp_files})
target_link_libraries(am_super)
ament_target_dependencies(am_super ${dependencies})

add_executable(resource_monitor src/resource_monitor/resource_status_class.cpp
                                src/resource_monitor/resource_monitor_main.cpp
                                src/resource_monitor/resource_monitor_node.cpp)
ament_target_dependencies(resource_monitor ${dependencies})

install(DIRECTORY include/
        DESTINATION include/
)

install(TARGETS
  am_super
  resource_monitor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ros_testing REQUIRED)

  # the following lines skip linters
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Run all lint tests in package.xml except those listed above
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest_executable(abort_to_disarming
    super_test/abort_to_disarming/abort_to_disarming_rostest.cpp
  )

  target_link_libraries(abort_to_disarming)
  ament_target_dependencies(abort_to_disarming ${dependencies})
  add_ros_test(super_test/abort_to_disarming/launch/abort_to_disarming.test.py TIMEOUT 120 ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()

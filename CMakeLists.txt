cmake_minimum_required(VERSION 3.8)
project(mynteye_get_data)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(dependencies
  ament_cmake
  rclcpp
  sensor_msgs
  geometry_msgs
  image_proc
  # ros2_aruco
  ros2_aruco_interfaces
  )

# find dependencies
foreach(dependency ${dependencies})
  find_package(${dependency} REQUIRED)
endforeach()

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
DESTINATION share/${PROJECT_NAME})

add_executable(aruco_mean src/aruco_mean.cpp)
ament_target_dependencies(aruco_mean ${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

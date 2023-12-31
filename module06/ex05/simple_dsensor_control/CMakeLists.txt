cmake_minimum_required(VERSION 3.8)
project(simple_dsensor_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


add_executable(simple_dsensor src/depth_sensor_stopper.cpp)
add_executable(simple_dsensor_points src/depth_sensor_stopper.cpp)
ament_target_dependencies(simple_dsensor rclcpp geometry_msgs sensor_msgs)
ament_target_dependencies(simple_dsensor_points rclcpp geometry_msgs sensor_msgs)

install(TARGETS
simple_dsensor
simple_dsensor_points
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

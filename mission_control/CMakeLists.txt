cmake_minimum_required(VERSION 3.8)
project(mission_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(delta_interfaces REQUIRED)

add_executable(mission_control src/mission_control.cpp)
ament_target_dependencies(mission_control rclcpp delta_interfaces)

install(TARGETS mission_control
  DESTINATION lib/${PROJECT_NAME})

ament_package()

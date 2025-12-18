cmake_minimum_required(VERSION 3.16)
project(mitsubishi_robot_hardware LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Boost REQUIRED)

# Library
add_library(
  ${PROJECT_NAME} 
  SHARED
  src/mitsubishi_robot_hardware.cpp
)
target_compile_definitions(
  ${PROJECT_NAME}
  PUBLIC
  cxx_std_17
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  Boost
)

# Pluginlib
pluginlib_export_plugin_description_file(hardware_interface mitsubishi_robot_hardware.xml)

# Install
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Exports
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)

ament_package()

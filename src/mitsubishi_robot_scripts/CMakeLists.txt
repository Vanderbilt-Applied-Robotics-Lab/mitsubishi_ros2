cmake_minimum_required(VERSION 3.5)
project(mitsubishi_robot_scripts)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()


# Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Include(s)
include_directories(include)

# Executable(s)

# Install executables
install(
  PROGRAMS
    scripts/svg_path_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files

ament_package()

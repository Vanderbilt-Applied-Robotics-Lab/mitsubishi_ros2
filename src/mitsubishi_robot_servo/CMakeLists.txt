cmake_minimum_required(VERSION 3.8)
project(mitsubishi_robot_servo)

# warnings are errors
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp)
find_package(geometry_msgs)
find_package(moveit_msgs)
find_package(control_msgs)
find_package(std_srvs)
find_package(mitsubishi_robot_msgs)

# include
include_directories(include)

# executable
add_executable(servo src/servo.cpp)
ament_target_dependencies(servo rclcpp geometry_msgs moveit_msgs control_msgs std_srvs mitsubishi_robot_msgs)

# install
install(
  TARGETS servo
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

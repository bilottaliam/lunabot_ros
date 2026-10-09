cmake_minimum_required(VERSION 3.10)
project(lunabot_teleop)

# Configure compiler definitions
add_definitions(-w)

# Set default C standard
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Set default C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Enable compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lunabot_msgs REQUIRED)
find_package(Threads REQUIRED)  # sparkcan links Threads::Threads but does not find it
# sparkcan exports Threads::Threads without finding it
find_package(Threads REQUIRED)
find_package(sparkcan REQUIRED)
find_package(lunabot_logger REQUIRED)

# Define build targets
add_executable(controller_teleop src/controller_teleop.cpp)
add_executable(motor_controller src/motor_controller.cpp)

# ament_target_dependencies() was removed in Lyrical; link the exported targets instead
target_link_libraries(
  controller_teleop
  ${rclcpp_TARGETS}
  ${rclcpp_action_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${lunabot_msgs_TARGETS}
  sparkcan::sparkcan)
target_include_directories(controller_teleop PRIVATE ${lunabot_logger_INCLUDE_DIRS})

target_link_libraries(
  motor_controller
  ${rclcpp_TARGETS}
  ${std_msgs_TARGETS}
  ${lunabot_msgs_TARGETS}
  sparkcan::sparkcan)
target_include_directories(motor_controller PRIVATE ${lunabot_logger_INCLUDE_DIRS})

# Install package files
install(TARGETS controller_teleop motor_controller DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS src/keyboard_teleop.py DESTINATION lib/${PROJECT_NAME})

ament_package()

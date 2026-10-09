cmake_minimum_required(VERSION 3.10)
project(lunabot_nav)

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
find_package(geometry_msgs REQUIRED)
find_package(lunabot_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Threads REQUIRED)  # sparkcan links Threads::Threads but does not find it
# sparkcan exports Threads::Threads without finding it
find_package(Threads REQUIRED)
find_package(sparkcan REQUIRED)
find_package(lunabot_logger REQUIRED)

set(DEPENDENCIES
    geometry_msgs
    lunabot_msgs
    lunabot_logger
    nav2_core
    nav2_costmap_2d
    nav2_msgs
    nav2_util
    nav_msgs
    pluginlib
    rclcpp
    rclcpp_action
    sensor_msgs
    tf2_ros
    sparkcan)

set(SIM_DEPENDENCIES
    geometry_msgs
    lunabot_msgs
    lunabot_logger
    nav2_core
    nav2_costmap_2d
    nav2_msgs
    nav2_util
    nav_msgs
    pluginlib
    rclcpp
    rclcpp_action
    sensor_msgs
    tf2_ros)

# ament_target_dependencies() was removed in Lyrical; link the exported targets instead
foreach(dep ${DEPENDENCIES})
  list(APPEND DEPENDENCIES_TARGETS ${${dep}_TARGETS})
endforeach()
foreach(dep ${SIM_DEPENDENCIES})
  list(APPEND SIM_DEPENDENCIES_TARGETS ${${dep}_TARGETS})
endforeach()

# Define build targets
add_executable(assisted_excavation_server src/assisted_excavation_server.cpp)
add_executable(auto_excavation_server src/auto_excavation_server.cpp)
add_executable(assisted_depositing_server src/assisted_depositing_server.cpp)
add_executable(auto_depositing_server src/auto_depositing_server.cpp)
add_executable(navigation_client src/navigation_client.cpp)

add_executable(excavation_server_sim src/excavation_server_sim.cpp)
add_executable(depositing_server_sim src/depositing_server_sim.cpp)

target_link_libraries(assisted_excavation_server ${DEPENDENCIES_TARGETS} sparkcan::sparkcan)
target_include_directories(assisted_excavation_server PRIVATE ${lunabot_logger_INCLUDE_DIRS})
target_link_libraries(auto_excavation_server ${DEPENDENCIES_TARGETS} sparkcan::sparkcan)
target_include_directories(auto_excavation_server PRIVATE ${lunabot_logger_INCLUDE_DIRS})
target_link_libraries(assisted_depositing_server ${DEPENDENCIES_TARGETS} sparkcan::sparkcan)
target_include_directories(assisted_depositing_server PRIVATE ${lunabot_logger_INCLUDE_DIRS})
target_link_libraries(auto_depositing_server ${DEPENDENCIES_TARGETS} sparkcan::sparkcan)
target_include_directories(auto_depositing_server PRIVATE ${lunabot_logger_INCLUDE_DIRS})
target_link_libraries(navigation_client ${DEPENDENCIES_TARGETS} sparkcan::sparkcan)
target_include_directories(navigation_client PRIVATE ${lunabot_logger_INCLUDE_DIRS})

target_link_libraries(excavation_server_sim ${SIM_DEPENDENCIES_TARGETS})
target_include_directories(excavation_server_sim PRIVATE ${lunabot_logger_INCLUDE_DIRS})
target_link_libraries(depositing_server_sim ${SIM_DEPENDENCIES_TARGETS})
target_include_directories(depositing_server_sim PRIVATE ${lunabot_logger_INCLUDE_DIRS})

# Install package files
install(
  TARGETS assisted_excavation_server
          auto_excavation_server
          assisted_depositing_server
          auto_depositing_server
          navigation_client
          excavation_server_sim
          depositing_server_sim
  DESTINATION lib/${PROJECT_NAME})

ament_package()

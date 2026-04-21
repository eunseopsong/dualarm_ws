cmake_minimum_required(VERSION 3.8)
project(dualarm_forcecon)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pinocchio REQUIRED)

find_package(dualarm_forcecon_interfaces REQUIRED)
find_package(dualarm_kinematics REQUIRED)

# ------------------------------------------------------------------------------
# Executable
# ------------------------------------------------------------------------------
add_executable(dualarm_forcecon_node
  src/node_dualarm_main.cpp
  src/DualArmForceControl.cpp
  src/states_current_callback_dualarm.cpp
  src/states_target_callback_dualarm.cpp
)

target_include_directories(dualarm_forcecon_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(dualarm_forcecon_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  dualarm_forcecon_interfaces
  dualarm_kinematics
)

target_link_libraries(dualarm_forcecon_node
  Eigen3::Eigen
  yaml-cpp
)

target_compile_features(dualarm_forcecon_node PUBLIC cxx_std_17)

# ------------------------------------------------------------------------------
# Install
# ------------------------------------------------------------------------------
install(
  TARGETS dualarm_forcecon_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY yaml/
  DESTINATION share/${PROJECT_NAME}/yaml
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
endif()

# ------------------------------------------------------------------------------
# Export
# ------------------------------------------------------------------------------
ament_export_include_directories(include)

ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  dualarm_forcecon_interfaces
  dualarm_kinematics
)

ament_package()

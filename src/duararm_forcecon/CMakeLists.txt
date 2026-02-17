cmake_minimum_required(VERSION 3.8)
project(dualarm_forcecon)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(dualarm_ctrl
  src/node_dualarm_main.cpp
  src/DualArmForceControl.cpp
  src/states_callback_dualarm.cpp
)

ament_target_dependencies(dualarm_ctrl
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)

target_link_libraries(dualarm_ctrl Eigen3::Eigen)

install(TARGETS dualarm_ctrl DESTINATION lib/${PROJECT_NAME})
ament_package()
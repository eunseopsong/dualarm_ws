cmake_minimum_required(VERSION 3.8)
project(dualarm_forcecon)

# =========================
# C++ standard
# =========================
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# =========================
# Dependencies
# =========================
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2_kdl REQUIRED)

find_package(pinocchio REQUIRED)

# =========================
# Target
# =========================
add_executable(dualarm_forcecon_node
  src/node_dualarm_main.cpp
  src/DualArmForceControl.cpp
  src/states_current_callback_dualarm.cpp
  src/states_target_callback_dualarm.cpp
)

target_include_directories(dualarm_forcecon_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dualarm_forcecon_node
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  kdl_parser
  orocos_kdl
  urdf
  tf2_kdl
  pinocchio
)

# Eigen
target_link_libraries(dualarm_forcecon_node
  Eigen3::Eigen
  orocos-kdl
)

# Pinocchio: ament에는 "pinocchio"로, 링크는 target로
if(TARGET pinocchio::pinocchio)
  target_link_libraries(dualarm_forcecon_node pinocchio::pinocchio)
elseif(DEFINED pinocchio_LIBRARIES)
  target_link_libraries(dualarm_forcecon_node ${pinocchio_LIBRARIES})
  if(DEFINED pinocchio_INCLUDE_DIRS)
    target_include_directories(dualarm_forcecon_node PUBLIC ${pinocchio_INCLUDE_DIRS})
  endif()
endif()

# =========================
# Install
# =========================
install(TARGETS dualarm_forcecon_node
  DESTINATION lib/${PROJECT_NAME}
)

# include/dualarm_forcecon/** -> install/include/dualarm_forcecon/**
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()

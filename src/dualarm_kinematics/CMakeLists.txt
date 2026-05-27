cmake_minimum_required(VERSION 3.8)
project(dualarm_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(pinocchio REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Pure CMake dependencies used by header-only inline implementations.
target_link_libraries(${PROJECT_NAME} INTERFACE
  Eigen3::Eigen
)

if(TARGET yaml-cpp::yaml-cpp)
  target_link_libraries(${PROJECT_NAME} INTERFACE yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
  target_link_libraries(${PROJECT_NAME} INTERFACE yaml-cpp)
elseif(YAML_CPP_LIBRARIES)
  target_include_directories(${PROJECT_NAME} INTERFACE ${YAML_CPP_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} INTERFACE ${YAML_CPP_LIBRARIES})
endif()

if(TARGET pinocchio::pinocchio)
  target_link_libraries(${PROJECT_NAME} INTERFACE pinocchio::pinocchio)
elseif(TARGET pinocchio)
  target_link_libraries(${PROJECT_NAME} INTERFACE pinocchio)
else()
  target_include_directories(${PROJECT_NAME} INTERFACE ${pinocchio_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} INTERFACE ${pinocchio_LIBRARIES})
endif()

# ROS/ament dependencies.
ament_target_dependencies(${PROJECT_NAME} INTERFACE
  geometry_msgs
  orocos_kdl
  kdl_parser
  urdf
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  FILES
    urdf/rby1a_kdl.urdf
    urdf/aidin_dsr_dualarm.urdf
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  geometry_msgs
  orocos_kdl
  kdl_parser
  urdf
)

ament_package()

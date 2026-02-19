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

# v5 추가 라이브러리 로드
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# include 폴더 경로 추가 (이 안에 있는 헤더 파일들을 참조할 수 있게 함)
# 회원님이 만드신 Kinematics 폴더도 여기서 자동으로 인식됩니다.
include_directories(
  include
  ${orocos_kdl_INCLUDE_DIRS}
)

# 헤더 온리(.hpp) 방식을 사용하므로 src 폴더에 있는 기존 파일 3개만 빌드합니다.
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
  kdl_parser
  orocos_kdl
  urdf
  tf2_kdl
  geometry_msgs
  nav_msgs
)

target_link_libraries(dualarm_ctrl Eigen3::Eigen)

install(TARGETS dualarm_ctrl DESTINATION lib/${PROJECT_NAME})

# include 폴더 설치 규칙 (다른 패키지에서 헤더를 참조할 경우 대비)
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
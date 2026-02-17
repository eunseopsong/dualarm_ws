cmake_minimum_required(VERSION 3.8)
# [중요] package.xml의 <name>과 일치해야 함
project(dualarm_forcecon)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 의존성 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# 2. Include 경로 설정
include_directories(
  include
)

# 3. 실행 파일 빌드
add_executable(dualarm_ctrl
  src/node_dualarm_main.cpp
  src/DualArmForceControl.cpp  # 클래스 소스 파일 추가
)

# 4. 의존성 링크
target_link_libraries(dualarm_ctrl Eigen3::Eigen)
ament_target_dependencies(dualarm_ctrl
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

# 5. 설치 규칙
install(TARGETS dualarm_ctrl
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
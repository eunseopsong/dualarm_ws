cmake_minimum_required(VERSION 3.8)
project(dualarm_forcecon)

# Default to C++14 or 17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =============================================================================
# 1. Find Dependencies
# =============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# =============================================================================
# 2. Include Directories
# =============================================================================
include_directories(include)
# Eigen3 헤더 경로 추가
include_directories(${Eigen3_INCLUDE_DIRS})

# =============================================================================
# 3. Build Executable
# =============================================================================
add_executable(dualarm_ctrl
  src/node_dualarm_main.cpp
  src/DualArmForceControl.cpp
  src/states_callback_dualarm.cpp  # <--- [중요] 이 파일이 꼭 포함되어야 합니다!
)

# ROS 2 의존성 링크
ament_target_dependencies(dualarm_ctrl
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
)

# Eigen3 링크 (ament_target_dependencies에 포함되지 않을 경우 대비 명시적 링크)
target_link_libraries(dualarm_ctrl Eigen3::Eigen)

# =============================================================================
# 4. Install Rules
# =============================================================================
# 실행 파일 설치
install(TARGETS dualarm_ctrl
  DESTINATION lib/${PROJECT_NAME}
)

# 헤더 파일 설치 (선택 사항이지만 권장)
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
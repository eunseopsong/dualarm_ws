cmake_minimum_required(VERSION 3.8)
project(rby1_joystick_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================
# Dependencies
# =========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# =========================
# Build executable
# =========================
add_executable(rby1_joystick_teleop_node
  src/rby1_joystick_teleop_node.cpp
)

target_compile_features(rby1_joystick_teleop_node PUBLIC cxx_std_17)

ament_target_dependencies(rby1_joystick_teleop_node
  rclcpp
  sensor_msgs
)

add_executable(rby1_wheel_teleop_node
  src/rby1_wheel_teleop_node.cpp
)

target_compile_features(rby1_wheel_teleop_node PUBLIC cxx_std_17)

ament_target_dependencies(rby1_wheel_teleop_node
  rclcpp
  sensor_msgs
)


# =========================
# Install
# =========================
install(TARGETS
  rby1_joystick_teleop_node
  rby1_wheel_teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

# Optional folders
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
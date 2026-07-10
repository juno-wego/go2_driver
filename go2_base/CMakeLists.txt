cmake_minimum_required(VERSION 3.8)
project(go2_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(go2_interface REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_go REQUIRED)

add_executable(go2_cmd_vel_bridge src/go2_cmd_vel_bridge.cpp)
ament_target_dependencies(go2_cmd_vel_bridge
  geometry_msgs
  go2_interface
  rclcpp
  unitree_api
  unitree_go
)

add_executable(go2_state_bridge src/go2_state_bridge.cpp)
ament_target_dependencies(go2_state_bridge
  geometry_msgs
  go2_interface
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2_ros
  unitree_go
)

install(TARGETS
  go2_cmd_vel_bridge
  go2_state_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/set_unitree_static_ip.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

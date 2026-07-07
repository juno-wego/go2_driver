cmake_minimum_required(VERSION 3.8)
project(go2_base)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python packages
ament_python_install_package(go2_base)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts/executables
install(PROGRAMS
  go2_base/go2_driver_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

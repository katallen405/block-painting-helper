cmake_minimum_required(VERSION 3.8)
project(nav_to_goal)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(bph_interfaces REQUIRED)
# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Install config files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

# Install Python scripts (entry points defined in setup.py)
install(PROGRAMS
  nav_to_goal/navigator_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME navigator_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

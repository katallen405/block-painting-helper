cmake_minimum_required(VERSION 3.20)
project(bph_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveToPose.srv"
  "action/BphPickmeup.action"
  DEPENDENCIES geometry_msgs
)

ament_package()

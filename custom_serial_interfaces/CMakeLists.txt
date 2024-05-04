cmake_minimum_required(VERSION 3.8)
project(custom_serial_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Receive.msg"
  "srv/SendPackage.srv"
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

ament_package()

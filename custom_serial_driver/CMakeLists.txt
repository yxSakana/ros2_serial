cmake_minimum_required(VERSION 3.8)
project(custom_serial_driver)

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARN 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

find_package(Threads REQUIRED)
find_package(fmt REQUIRED)

target_link_libraries(${PROJECT_NAME}
  Threads::Threads
  fmt::fmt
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "custom_serial::SerialDriverNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)

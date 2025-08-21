cmake_minimum_required(VERSION 3.5)

project(ros_gz_example_application)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
add_executable(controller src/control/controller.cpp src/control/pid.cpp)
ament_target_dependencies(controller rclcpp sensor_msgs geometry_msgs nav_msgs)

target_include_directories(controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
install(
DIRECTORY include/
DESTINATION include
)

install(TARGETS
  controller
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()

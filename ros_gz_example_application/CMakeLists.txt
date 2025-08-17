cmake_minimum_required(VERSION 3.5)

project(ros_gz_example_application)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
add_executable(controller src/control/controller.cpp)
ament_target_dependencies(controller rclcpp sensor_msgs geometry_msgs)

install(TARGETS
  controller
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()

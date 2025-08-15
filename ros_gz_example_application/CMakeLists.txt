cmake_minimum_required(VERSION 3.5)

project(ros_gz_example_application)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
add_executable(velocity_publisher src/control/velocity.cpp)
ament_target_dependencies(velocity_publisher rclcpp geometry_msgs)

install(TARGETS
  velocity_publisher
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()

cmake_minimum_required(VERSION 3.5)

project(ros_gz_example_application)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
add_executable(velocity_publisher src/control/velocity.cpp)
add_executable(laser_subscriber src/control/perception.cpp)
ament_target_dependencies(velocity_publisher  rclcpp geometry_msgs )
ament_target_dependencies(laser_subscriber rclcpp sensor_msgs)

install(TARGETS
  velocity_publisher
  laser_subscriber
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()

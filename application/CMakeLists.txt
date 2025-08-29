cmake_minimum_required(VERSION 3.5)
project(application)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)  # Changed from eigen3 to Eigen3

add_executable(controller
  src/control/controller.cpp
  src/control/pid.cpp
  src/control/potential_field.cpp
  src/control/wrap_angle.cpp)

ament_target_dependencies(controller rclcpp sensor_msgs geometry_msgs nav_msgs)

target_include_directories(controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Eigen3_INCLUDE_DIRS}  # Changed from eigen3_INCLUDE_DIRS
)

# Link Eigen3 if needed
target_link_libraries(controller Eigen3::Eigen)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  controller
  DESTINATION lib/${PROJECT_NAME}
)

# Optional: Add testing support if you want to use ament_lint_auto
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
